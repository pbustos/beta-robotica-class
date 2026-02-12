/*
 * GridESDF - Sparse Obstacle Grid with Euclidean Signed Distance Field
 *
 * Implementation following VoxBlox philosophy:
 * - Sparse storage: only obstacle cells are stored
 * - ESDF computed incrementally via Dijkstra multi-source
 * - Free space is implicit
 */

#include "grid_esdf.h"
#include <algorithm>
#include <cmath>

GridESDF::~GridESDF()
{
    reset();
}

void GridESDF::initialize(int tile_size, QGraphicsScene *scene)
{
    params_.tile_size = tile_size;
    scene_ = scene;
    clear();
}

void GridESDF::clear()
{
    // Remove visual items
    for (auto &[k, cell] : obstacles_)
    {
        if (cell.tile && scene_)
        {
            scene_->removeItem(cell.tile);
            delete cell.tile;
        }
    }
    for (auto *tile : inflation_tiles_)
    {
        if (scene_) scene_->removeItem(tile);
        delete tile;
    }

    obstacles_.clear();
    esdf_cache_.clear();
    inflation_tiles_.clear();
}

void GridESDF::reset()
{
    clear();
}

//////////////////////////////////////////////////////////////////////////////
// Key conversion
//////////////////////////////////////////////////////////////////////////////

GridESDF::Key GridESDF::point_to_key(const Eigen::Vector2f &p) const
{
    return point_to_key(p.x(), p.y());
}

GridESDF::Key GridESDF::point_to_key(float x, float y) const
{
    const float ts = static_cast<float>(params_.tile_size);
    return {static_cast<int>(std::floor(x / ts) * ts),
            static_cast<int>(std::floor(y / ts) * ts)};
}

Eigen::Vector2f GridESDF::key_to_point(const Key &k) const
{
    const float half_tile = params_.tile_size / 2.0f;
    return {static_cast<float>(k.first) + half_tile,
            static_cast<float>(k.second) + half_tile};
}

//////////////////////////////////////////////////////////////////////////////
// Obstacle management
//////////////////////////////////////////////////////////////////////////////

bool GridESDF::is_obstacle(const Key &k) const
{
    auto it = obstacles_.find(k);
    if (it == obstacles_.end())
        return false;

    // Fast path: confirmed obstacle with visual tile
    if (it->second.tile != nullptr)
        return true;

    // Secondary check: enough hits and confidence (for planning safety even without visual)
    return it->second.hits >= 2.0f && it->second.log_odds >= params_.occupancy_threshold * 0.5f;
}

// Check if a cell should be avoided for path planning (obstacle + inflation zone)
bool GridESDF::is_occupied_for_planning(const Key &k, float robot_radius) const
{
    // First check if it's a direct obstacle (fast path)
    auto it = obstacles_.find(k);
    if (it != obstacles_.end() && it->second.tile != nullptr)
        return true;

    // If no robot radius, only check direct obstacle
    if (robot_radius <= 0)
        return false;

    // Use ESDF cache if available for faster lookup
    auto esdf_it = esdf_cache_.find(k);
    if (esdf_it != esdf_cache_.end() && esdf_it->second.distance < robot_radius)
        return true;

    // Check only immediate neighbors (1-ring) for robot radius
    // This is O(8) instead of O(cells_to_check^2)
    const int cells_to_check = std::max(1, static_cast<int>(std::ceil(robot_radius / static_cast<float>(params_.tile_size))));

    // Optimized search: only check in cardinal and diagonal directions
    static const std::array<std::pair<int,int>, 8> directions = {{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    }};

    for (int dist = 1; dist <= cells_to_check; ++dist)
    {
        for (const auto &[dx_base, dy_base] : directions)
        {
            int dx = dx_base * dist;
            int dy = dy_base * dist;

            Key neighbor{k.first + dx * params_.tile_size,
                        k.second + dy * params_.tile_size};

            auto nit = obstacles_.find(neighbor);
            if (nit != obstacles_.end() && nit->second.tile != nullptr)
            {
                float actual_dist = std::hypot(static_cast<float>(dx), static_cast<float>(dy))
                                   * static_cast<float>(params_.tile_size);
                if (actual_dist <= robot_radius)
                    return true;
            }
        }
    }
    return false;
}

bool GridESDF::is_free(const Key &k) const
{
    return !is_obstacle(k);  // Sparse assumption: not in map = free
}

void GridESDF::add_obstacle(const Key &k, uint64_t timestamp)
{
    auto it = obstacles_.find(k);
    if (it != obstacles_.end())
    {
        // Update existing obstacle
        auto &cell = it->second;
        cell.hits += 1.0f;
        cell.log_odds = std::min(cell.log_odds + params_.log_odds_hit, params_.log_odds_max);
        cell.last_seen = timestamp;

        // Create visual tile only when confidence is high enough AND has neighbor support
        if (cell.tile == nullptr &&
            cell.hits >= params_.min_hits_to_confirm &&
            cell.log_odds >= params_.occupancy_threshold &&
            scene_)
        {
            // Check if at least one neighbor also has hits (avoid isolated noise)
            bool has_neighbor_support = false;
            for (const auto &neighbor : get_neighbors_8(k))
            {
                auto nit = obstacles_.find(neighbor);
                if (nit != obstacles_.end() && nit->second.hits >= 2.0f)
                {
                    has_neighbor_support = true;
                    break;
                }
            }

            if (has_neighbor_support)
            {
                const float ts = params_.tile_size;
                cell.tile = scene_->addRect(-ts/2, -ts/2, ts, ts,
                                            QPen(Qt::NoPen),
                                            QBrush(QColor(params_.obstacle_color)));
                cell.tile->setPos(k.first + ts/2, k.second + ts/2);
                cell.tile->setZValue(1);
                visualization_dirty_ = true;  // Mark for inflation update
            }
        }
    }
    else
    {
        // New obstacle - don't create visual yet, wait for confirmation
        ObstacleCell cell;
        cell.log_odds = params_.log_odds_hit;
        cell.hits = 1.0f;
        cell.last_seen = timestamp;
        cell.tile = nullptr;  // No visual until confirmed

        obstacles_[k] = cell;
        // Note: ESDF cache is NOT cleared here - it will be updated incrementally
        // when the obstacle is confirmed and propagate_esdf_from is called
    }
}

void GridESDF::add_confirmed_obstacle(const Key &k)
{
    // Add obstacle directly as confirmed - used for external map loading
    // Skip neighbor verification, create visual tile immediately
    auto it = obstacles_.find(k);
    if (it != obstacles_.end())
    {
        // Already exists - ensure it has a tile
        if (it->second.tile == nullptr && scene_)
        {
            const float ts = params_.tile_size;
            it->second.tile = scene_->addRect(-ts/2, -ts/2, ts, ts,
                                              QPen(Qt::NoPen),
                                              QBrush(QColor(params_.obstacle_color)));
            it->second.tile->setPos(k.first + ts/2, k.second + ts/2);
            it->second.tile->setZValue(1);
        }
        it->second.log_odds = params_.log_odds_max;
        it->second.hits = params_.min_hits_to_confirm + 1;
    }
    else
    {
        // New obstacle - create as fully confirmed with visual
        ObstacleCell cell;
        cell.log_odds = params_.log_odds_max;
        cell.hits = params_.min_hits_to_confirm + 1;
        cell.last_seen = 0;

        if (scene_)
        {
            const float ts = params_.tile_size;
            cell.tile = scene_->addRect(-ts/2, -ts/2, ts, ts,
                                        QPen(Qt::NoPen),
                                        QBrush(QColor(params_.obstacle_color)));
            cell.tile->setPos(k.first + ts/2, k.second + ts/2);
            cell.tile->setZValue(1);
        }
        obstacles_[k] = cell;
    }
    // Note: visualization_dirty_ is NOT set here to avoid performance issues
    // when loading many obstacles. Call mark_visualization_dirty() after batch loading.
}

void GridESDF::remove_obstacle(const Key &k)
{
    auto it = obstacles_.find(k);
    if (it != obstacles_.end())
    {
        if (it->second.tile && scene_)
        {
            scene_->removeItem(it->second.tile);
            delete it->second.tile;
            visualization_dirty_ = true;  // Mark for inflation update
        }
        obstacles_.erase(it);

        // Invalidate ESDF cache
        invalidate_esdf_near(k);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Map update from LiDAR
//////////////////////////////////////////////////////////////////////////////

void GridESDF::update(const std::vector<Eigen::Vector2f> &hit_points,
                      const Eigen::Vector2f &robot_pos,
                      float max_range,
                      uint64_t timestamp)
{
    if (hit_points.empty()) return;

    const float range_threshold = max_range * 0.95f;
    const float tile_size_f = static_cast<float>(params_.tile_size);

    // Track keys we've already processed this frame to avoid duplicates
    std::unordered_set<Key, boost::hash<Key>> processed_keys;
    processed_keys.reserve(hit_points.size());

    for (const auto &point : hit_points)
    {
        const float range = (point - robot_pos).norm();

        // Skip invalid points
        if (range < tile_size_f) continue;

        const bool is_max_range = (range >= range_threshold);

        if (!is_max_range)
        {
            // This is a real obstacle hit
            Key k = point_to_key(point);
            // Only process each key once per frame
            if (processed_keys.insert(k).second)
            {
                add_obstacle(k, timestamp);
            }
        }

        // Trace free space along ray (sparse sampling)
        trace_free_space(robot_pos, point, timestamp);
    }
    // Note: ESDF propagation is NOT done here - it's computed on-demand when needed
    // for path planning via get_distance() or compute_path()
}

void GridESDF::trace_free_space(const Eigen::Vector2f &from,
                                 const Eigen::Vector2f &to,
                                 uint64_t timestamp)
{
    // Skip if obstacles map is empty (nothing to clear)
    if (obstacles_.empty()) return;

    // Sparse sampling: check every 2-3 cells along the ray
    const float tile_size_f = static_cast<float>(params_.tile_size);
    const float step = tile_size_f * 2.5f;  // Sample every ~2.5 cells

    const Eigen::Vector2f delta = to - from;
    const float length = delta.norm() - tile_size_f;  // Don't clear the hit cell
    if (length <= 0) return;

    const Eigen::Vector2f dir = delta.normalized();

    // Track last key to avoid checking same cell twice
    Key last_key = {std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};

    for (float t = tile_size_f; t < length; t += step)
    {
        const Eigen::Vector2f sample = from + dir * t;
        const Key k = point_to_key(sample);

        // Skip if same cell as last sample
        if (k == last_key) continue;
        last_key = k;

        // If this cell is marked as obstacle, reduce its confidence
        auto it = obstacles_.find(k);
        if (it != obstacles_.end())
        {
            auto &cell = it->second;
            cell.log_odds += params_.log_odds_miss;

            // Remove obstacle if confidence drops below threshold
            if (cell.log_odds < params_.free_threshold)
            {
                remove_obstacle(k);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
// ESDF computation
//////////////////////////////////////////////////////////////////////////////

void GridESDF::propagate_esdf_from(const std::vector<Key> &seeds)
{
    const float tile_size_f = static_cast<float>(params_.tile_size);
    const float max_dist = params_.max_esdf_distance;

    // Priority queue: (distance, key)
    using QElement = std::pair<float, Key>;
    std::priority_queue<QElement, std::vector<QElement>, std::greater<>> pq;

    // Initialize seeds
    for (const auto &k : seeds)
    {
        esdf_cache_[k] = {0.0f, k};
        pq.push({0.0f, k});
    }

    // Dijkstra propagation
    while (!pq.empty())
    {
        auto [dist, current] = pq.top();
        pq.pop();

        // Skip if we already have a better distance
        auto it = esdf_cache_.find(current);
        if (it != esdf_cache_.end() && dist > it->second.distance)
            continue;

        // Stop if we've exceeded max distance
        if (dist > max_dist)
            continue;

        // Propagate to neighbors
        for (const auto &neighbor : get_neighbors_4(current))
        {
            const float new_dist = dist + tile_size_f;

            auto nit = esdf_cache_.find(neighbor);
            if (nit == esdf_cache_.end() || new_dist < nit->second.distance)
            {
                esdf_cache_[neighbor] = {new_dist, esdf_cache_[current].nearest_obstacle};
                pq.push({new_dist, neighbor});
            }
        }
    }
}

void GridESDF::invalidate_esdf_near(const Key &removed_obstacle)
{
    // Simple and fast: just remove the entry for the removed obstacle
    // The ESDF will be recomputed on-demand when needed
    esdf_cache_.erase(removed_obstacle);
    // Note: We don't invalidate neighbors - the ESDF cache is lazy
    // and will be recomputed when get_distance() is called
}

//////////////////////////////////////////////////////////////////////////////
// ESDF queries
//////////////////////////////////////////////////////////////////////////////

float GridESDF::get_distance(const Key &k)
{
    // Check if it's an obstacle
    if (is_obstacle(k))
        return 0.0f;

    // Check cache first - fast path
    auto it = esdf_cache_.find(k);
    if (it != esdf_cache_.end())
        return it->second.distance;

    // Not in cache - use fast local search within max_esdf_distance
    // This is O(1) average case instead of O(n)
    const float tile_f = static_cast<float>(params_.tile_size);
    const int search_radius = static_cast<int>(std::ceil(params_.max_esdf_distance / tile_f));

    float min_dist = params_.max_esdf_distance;
    Key nearest_obs = {0, 0};

    // Search in a spiral pattern from center outward for early termination
    for (int radius = 1; radius <= search_radius; ++radius)
    {
        const float min_possible_dist = static_cast<float>(radius) * tile_f;
        if (min_possible_dist >= min_dist)
            break;  // Can't find closer obstacle at this radius

        // Check cells at this radius
        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                // Only check cells on the perimeter
                if (std::abs(dx) != radius && std::abs(dy) != radius)
                    continue;

                Key neighbor = {k.first + dx * params_.tile_size,
                               k.second + dy * params_.tile_size};

                auto obs_it = obstacles_.find(neighbor);
                if (obs_it != obstacles_.end() && obs_it->second.tile != nullptr)
                {
                    float dist = std::hypot(static_cast<float>(dx), static_cast<float>(dy)) * tile_f;
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        nearest_obs = neighbor;
                    }
                }
            }
        }
    }

    // Cache the result
    esdf_cache_[k] = {min_dist, nearest_obs};
    return min_dist;
}

float GridESDF::get_distance(const Eigen::Vector2f &p)
{
    return get_distance(point_to_key(p));
}

//////////////////////////////////////////////////////////////////////////////
// Fast distance query (no cache, thread-safe, for localization)
//////////////////////////////////////////////////////////////////////////////

float GridESDF::get_distance_fast(const Key &k) const
{
    // Check if it's an obstacle
    auto obs_it = obstacles_.find(k);
    if (obs_it != obstacles_.end() && obs_it->second.tile != nullptr)
        return 0.0f;

    // Simple search: check only nearby cells (limited radius for speed)
    const float tile_f = static_cast<float>(params_.tile_size);
    const int search_radius = 5;  // Only check 5 cells away max (250mm at 50mm tiles)

    float min_dist = search_radius * tile_f;  // Return max if nothing found

    for (int dx = -search_radius; dx <= search_radius; ++dx)
    {
        for (int dy = -search_radius; dy <= search_radius; ++dy)
        {
            if (dx == 0 && dy == 0) continue;

            Key neighbor = {k.first + dx * params_.tile_size,
                           k.second + dy * params_.tile_size};

            auto it = obstacles_.find(neighbor);
            if (it != obstacles_.end() && it->second.tile != nullptr)
            {
                float dist = std::hypot(static_cast<float>(dx), static_cast<float>(dy)) * tile_f;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }
    }

    return min_dist;
}

float GridESDF::get_distance_fast(float x, float y) const
{
    return get_distance_fast(point_to_key(Eigen::Vector2f(x, y)));
}

float GridESDF::get_cost(const Key &k)
{
    if (is_obstacle(k))
        return params_.obstacle_cost;

    const float dist = get_distance(k);
    const float tile_size_f = static_cast<float>(params_.tile_size);

    // Exponential decay cost
    if (dist < tile_size_f * 2.0f)
        return params_.obstacle_cost * std::exp(-dist / tile_size_f);
    else if (dist < tile_size_f * 4.0f)
        return params_.obstacle_cost * 0.25f * std::exp(-(dist - tile_size_f * 2.0f) / (tile_size_f * 2.0f));
    else
        return params_.free_cost;
}

float GridESDF::get_cost(const Eigen::Vector2f &p)
{
    return get_cost(point_to_key(p));
}

//////////////////////////////////////////////////////////////////////////////
// Neighbors
//////////////////////////////////////////////////////////////////////////////

std::vector<GridESDF::Key> GridESDF::get_neighbors_4(const Key &k) const
{
    const int ts = params_.tile_size;
    return {
        {k.first + ts, k.second},
        {k.first - ts, k.second},
        {k.first, k.second + ts},
        {k.first, k.second - ts}
    };
}

std::vector<GridESDF::Key> GridESDF::get_neighbors_8(const Key &k) const
{
    const int ts = params_.tile_size;
    return {
        {k.first + ts, k.second},
        {k.first - ts, k.second},
        {k.first, k.second + ts},
        {k.first, k.second - ts},
        {k.first + ts, k.second + ts},
        {k.first + ts, k.second - ts},
        {k.first - ts, k.second + ts},
        {k.first - ts, k.second - ts}
    };
}

//////////////////////////////////////////////////////////////////////////////
// Path planning
//////////////////////////////////////////////////////////////////////////////

std::vector<Eigen::Vector2f> GridESDF::compute_path(const Eigen::Vector2f &source,
                                                     const Eigen::Vector2f &target,
                                                     float robot_radius,
                                                     float safety_factor)
{
    const Key source_key = point_to_key(source);
    const Key target_key = point_to_key(target);
    const float tile_size_f = static_cast<float>(params_.tile_size);

    // Clamp safety_factor to [0, 1]
    safety_factor = std::clamp(safety_factor, 0.f, 1.f);

    // Limit expansion to reasonable area - use configurable parameters
    const float direct_dist = std::hypot(static_cast<float>(target_key.first - source_key.first),
                                         static_cast<float>(target_key.second - source_key.second));
    const size_t max_nodes = std::max(params_.max_astar_nodes,
                                      static_cast<size_t>((direct_dist / tile_size_f) * params_.astar_distance_factor));

    qDebug() << "[A*] Computing path from" << source_key.first << source_key.second
             << "to" << target_key.first << target_key.second;

    // Check if source or target are in occupied zone (obstacle + inflation)
    if (is_occupied_for_planning(source_key, robot_radius))
    {
        qDebug() << "[A*] Source is in occupied zone!";
        return {};
    }
    if (is_occupied_for_planning(target_key, robot_radius))
    {
        qDebug() << "[A*] Target is in occupied zone!";
        return {};
    }

    // A* with ESDF-based cost - optimized with pre-reserved containers
    using QElement = std::pair<float, Key>;  // (f_score, key)
    std::priority_queue<QElement, std::vector<QElement>, std::greater<>> open_set;
    std::unordered_map<Key, float, boost::hash<Key>> g_score;
    std::unordered_map<Key, Key, boost::hash<Key>> came_from;
    std::unordered_set<Key, boost::hash<Key>> closed_set;  // Already processed nodes
    std::unordered_set<Key, boost::hash<Key>> occupied_cache;  // Cache for occupied cells

    // Pre-allocate reasonable sizes
    g_score.reserve(max_nodes / 2);
    came_from.reserve(max_nodes / 2);
    closed_set.reserve(max_nodes / 2);
    occupied_cache.reserve(max_nodes / 4);

    // Lambda with cached occupation check
    auto is_occupied_cached = [&](const Key &k) -> bool {
        // Check cache first
        if (occupied_cache.count(k) > 0)
            return true;
        if (closed_set.count(k) > 0)
            return false;  // Already processed as free

        bool occupied = is_occupied_for_planning(k, robot_radius);
        if (occupied)
            occupied_cache.insert(k);
        return occupied;
    };

    auto heuristic = [&](const Key &k) -> float {
        return std::hypot(static_cast<float>(k.first - target_key.first),
                         static_cast<float>(k.second - target_key.second));
    };

    g_score[source_key] = 0.0f;
    open_set.push({heuristic(source_key), source_key});

    // Pre-compute neighbor offsets to avoid repeated calculations
    static const std::array<std::tuple<int, int, float>, 8> neighbor_offsets = {{
        {1, 0, 1.0f}, {-1, 0, 1.0f}, {0, 1, 1.0f}, {0, -1, 1.0f},
        {1, 1, 1.414f}, {1, -1, 1.414f}, {-1, 1, 1.414f}, {-1, -1, 1.414f}
    }};

    while (!open_set.empty())
    {
        // Check node expansion limit
        if (closed_set.size() > max_nodes)
        {
            qDebug() << "[A*] Node expansion limit reached, no path found";
            return {};
        }

        auto [f, current] = open_set.top();
        open_set.pop();

        // Skip if already processed
        if (closed_set.count(current) > 0)
            continue;
        closed_set.insert(current);

        if (current == target_key)
        {
            // Reconstruct path
            std::vector<Eigen::Vector2f> path;
            path.reserve(closed_set.size() / 10);  // Reasonable estimate
            Key k = target_key;
            while (came_from.find(k) != came_from.end())
            {
                path.push_back(key_to_point(k));
                k = came_from[k];
            }
            path.push_back(key_to_point(source_key));
            std::reverse(path.begin(), path.end());

            qDebug() << "[A*] Path found with" << path.size() << "points, expanded" << closed_set.size() << "nodes";
            return path;
        }

        const float current_g = g_score[current];

        // Use pre-computed offsets instead of get_neighbors_8
        for (const auto &[dx, dy, dist_mult] : neighbor_offsets)
        {
            Key neighbor{current.first + dx * params_.tile_size,
                        current.second + dy * params_.tile_size};

            // Skip already processed nodes
            if (closed_set.count(neighbor) > 0)
                continue;

            // Skip occupied zones (cached check)
            if (is_occupied_cached(neighbor))
                continue;

            const float move_cost = tile_size_f * dist_mult;

            // Add ESDF-based cost only if safety_factor > 0 (skip expensive get_cost call)
            float tentative_g;
            if (safety_factor > 0.01f)
            {
                const float safety_scale = 1.0f + safety_factor * safety_factor * 10.0f;
                const float esdf_cost = get_cost(neighbor) * safety_factor * safety_scale;
                tentative_g = current_g + move_cost + esdf_cost;
            }
            else
            {
                tentative_g = current_g + move_cost;  // Pure shortest path
            }

            auto it = g_score.find(neighbor);
            if (it == g_score.end() || tentative_g < it->second)
            {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                open_set.push({tentative_g + heuristic(neighbor), neighbor});
            }
        }
    }

    qDebug() << "[A*] No path found after expanding" << closed_set.size() << "nodes";
    return {};  // No path found
}

bool GridESDF::is_line_of_sight_free(const Eigen::Vector2f &source,
                                      const Eigen::Vector2f &target,
                                      float robot_radius)
{
    const Eigen::Vector2f delta = target - source;
    const float length = delta.norm();
    if (length < 1e-6f) return true;

    const Eigen::Vector2f dir = delta.normalized();
    const float step = static_cast<float>(params_.tile_size) / 2.0f;

    // Check along the ray for obstacles
    for (float t = 0; t < length; t += step)
    {
        const Eigen::Vector2f p = source + dir * t;
        const Key k = point_to_key(p);

        // Check if this cell is an obstacle
        if (is_obstacle(k))
            return false;

        // Also check neighboring cells for robot_radius clearance
        if (robot_radius > 0)
        {
            // Check perpendicular offset for robot width
            const Eigen::Vector2f perp(-dir.y(), dir.x());
            for (float offset = -robot_radius; offset <= robot_radius; offset += step)
            {
                const Eigen::Vector2f check_p = p + perp * offset;
                if (is_obstacle(point_to_key(check_p)))
                    return false;
            }
        }
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////
// Serialization
//////////////////////////////////////////////////////////////////////////////

GridESDF::SerializedMap GridESDF::serialize_map() const
{
    SerializedMap map;
    map.tile_size = params_.tile_size;

    // Collect all confirmed obstacle cells and their inflation layers
    std::unordered_set<Key, boost::hash<Key>> processed;

    // First pass: add all confirmed obstacles (with tiles)
    for (const auto &[key, cell] : obstacles_)
    {
        if (cell.tile != nullptr)  // Confirmed obstacle
        {
            MapCell mc;
            mc.x = key.first;
            mc.y = key.second;
            mc.cost = 255;  // Maximum cost for obstacles
            map.cells.push_back(mc);
            processed.insert(key);
        }
    }

    // Second pass: add inflation layers (orange = layer1, green = layer2)
    std::unordered_set<Key, boost::hash<Key>> layer1_set;
    std::unordered_set<Key, boost::hash<Key>> layer2_set;

    // Compute layer 1 (adjacent to obstacles)
    for (const auto &[key, cell] : obstacles_)
    {
        if (cell.tile != nullptr)
        {
            for (const auto &n : get_neighbors_8(key))
            {
                if (processed.find(n) == processed.end())
                    layer1_set.insert(n);
            }
        }
    }

    // Add layer 1 cells with high cost
    for (const auto &key : layer1_set)
    {
        MapCell mc;
        mc.x = key.first;
        mc.y = key.second;
        mc.cost = 200;  // High cost for immediate neighbors
        map.cells.push_back(mc);
        processed.insert(key);
    }

    // Compute layer 2 (adjacent to layer 1)
    for (const auto &l1 : layer1_set)
    {
        for (const auto &n : get_neighbors_8(l1))
        {
            if (processed.find(n) == processed.end())
                layer2_set.insert(n);
        }
    }

    // Add layer 2 cells with medium cost
    for (const auto &key : layer2_set)
    {
        MapCell mc;
        mc.x = key.first;
        mc.y = key.second;
        mc.cost = 100;  // Medium cost for second layer
        map.cells.push_back(mc);
    }

    map.num_cells = static_cast<int32_t>(map.cells.size());

    return map;
}

//////////////////////////////////////////////////////////////////////////////
// Visualization
//////////////////////////////////////////////////////////////////////////////

void GridESDF::update_visualization(bool show_inflation)
{
    if (!scene_) return;

    // Count confirmed obstacles
    const size_t current_confirmed = count_confirmed_obstacles();

    // Skip if nothing has changed (fast path)
    if (!visualization_dirty_ && current_confirmed == last_confirmed_obstacles_)
        return;

    // Only update if obstacle count changed
    if (current_confirmed == last_confirmed_obstacles_ && !inflation_tiles_.empty())
    {
        visualization_dirty_ = false;
        return;
    }

    // Clear old inflation tiles
    for (auto *tile : inflation_tiles_)
    {
        scene_->removeItem(tile);
        delete tile;
    }
    inflation_tiles_.clear();

    if (!show_inflation)
    {
        visualization_dirty_ = false;
        last_confirmed_obstacles_ = current_confirmed;
        return;
    }

    const float ts = static_cast<float>(params_.tile_size);

    // Collect all confirmed obstacle keys first
    std::unordered_set<Key, boost::hash<Key>> obstacle_set;
    obstacle_set.reserve(obstacles_.size());
    for (const auto &[ok, cell] : obstacles_)
    {
        if (cell.tile != nullptr)
            obstacle_set.insert(ok);
    }

    // Layer 1 (orange): all cells adjacent to any obstacle
    std::unordered_set<Key, boost::hash<Key>> layer1_set;
    layer1_set.reserve(obstacle_set.size() * 4);
    for (const auto &ok : obstacle_set)
    {
        for (const auto &n : get_neighbors_8(ok))
        {
            // Add to layer1 if not an obstacle
            if (obstacle_set.find(n) == obstacle_set.end())
                layer1_set.insert(n);
        }
    }

    // Layer 2 (green): all cells adjacent to layer1 but not obstacle or layer1
    std::unordered_set<Key, boost::hash<Key>> layer2_set;
    layer2_set.reserve(layer1_set.size() * 4);
    for (const auto &l1 : layer1_set)
    {
        for (const auto &n : get_neighbors_8(l1))
        {
            // Add to layer2 if not an obstacle and not in layer1
            if (obstacle_set.find(n) == obstacle_set.end() &&
                layer1_set.find(n) == layer1_set.end())
                layer2_set.insert(n);
        }
    }

    // Reserve space for tiles
    inflation_tiles_.reserve(layer1_set.size() + layer2_set.size());

    // Draw layer 2 first (green) - lower z-order
    for (const auto &k : layer2_set)
    {
        auto *tile = scene_->addRect(-ts/2, -ts/2, ts, ts,
                                     QPen(Qt::NoPen),
                                     QBrush(QColor(params_.inflation_color_2)));
        tile->setPos(k.first + ts/2, k.second + ts/2);
        tile->setZValue(0);
        inflation_tiles_.push_back(tile);
    }

    // Draw layer 1 on top (orange) - higher z-order
    for (const auto &k : layer1_set)
    {
        auto *tile = scene_->addRect(-ts/2, -ts/2, ts, ts,
                                     QPen(Qt::NoPen),
                                     QBrush(QColor(params_.inflation_color_1)));
        tile->setPos(k.first + ts/2, k.second + ts/2);
        tile->setZValue(0.5);
        inflation_tiles_.push_back(tile);
    }

    visualization_dirty_ = false;
    last_confirmed_obstacles_ = current_confirmed;
}

size_t GridESDF::count_confirmed_obstacles() const
{
    size_t count = 0;
    for (const auto &[k, cell] : obstacles_)
    {
        if (cell.tile != nullptr)
            ++count;
    }
    return count;
}

//////////////////////////////////////////////////////////////////////////////
// Morphological post-processing
//////////////////////////////////////////////////////////////////////////////

void GridESDF::morphological_close(int iterations)
{
    // Closing = Dilation followed by Erosion
    // Effect: fills small holes and gaps in obstacles

    for (int iter = 0; iter < iterations; ++iter)
    {
        // Dilation: add cells where at least N neighbors are obstacles
        std::vector<Key> to_add;
        std::unordered_set<Key, boost::hash<Key>> obstacle_set;

        for (const auto &[k, cell] : obstacles_)
        {
            if (cell.tile != nullptr)
                obstacle_set.insert(k);
        }

        // Find candidates for dilation (neighbors of obstacles)
        std::unordered_set<Key, boost::hash<Key>> candidates;
        for (const auto &ok : obstacle_set)
        {
            for (const auto &n : get_neighbors_8(ok))
            {
                if (obstacle_set.find(n) == obstacle_set.end())
                    candidates.insert(n);
            }
        }

        // Add candidate if it has >= 3 obstacle neighbors (fills gaps in walls)
        for (const auto &c : candidates)
        {
            int obstacle_neighbors = 0;
            for (const auto &n : get_neighbors_8(c))
            {
                if (obstacle_set.find(n) != obstacle_set.end())
                    ++obstacle_neighbors;
            }
            if (obstacle_neighbors >= 3)
                to_add.push_back(c);
        }

        // Add the new obstacles
        uint64_t timestamp = 0;  // Use 0 for morphological additions
        for (const auto &k : to_add)
        {
            auto it = obstacles_.find(k);
            if (it == obstacles_.end())
            {
                // Create new confirmed obstacle directly
                ObstacleCell cell;
                cell.log_odds = params_.log_odds_max;
                cell.hits = params_.min_hits_to_confirm + 1;
                cell.last_seen = timestamp;

                if (scene_)
                {
                    const float ts = params_.tile_size;
                    cell.tile = scene_->addRect(-ts/2, -ts/2, ts, ts,
                                                QPen(Qt::NoPen),
                                                QBrush(QColor(params_.obstacle_color)));
                    cell.tile->setPos(k.first + ts/2, k.second + ts/2);
                    cell.tile->setZValue(1);
                }
                obstacles_[k] = cell;
            }
        }

        if (!to_add.empty())
            visualization_dirty_ = true;
    }
}

void GridESDF::morphological_open(int iterations)
{
    // Opening = Erosion followed by Dilation
    // Effect: removes isolated points and small protrusions

    for (int iter = 0; iter < iterations; ++iter)
    {
        // Erosion: remove obstacles with few neighbors
        std::vector<Key> to_remove;

        std::unordered_set<Key, boost::hash<Key>> obstacle_set;
        for (const auto &[k, cell] : obstacles_)
        {
            if (cell.tile != nullptr)
                obstacle_set.insert(k);
        }

        for (const auto &ok : obstacle_set)
        {
            int obstacle_neighbors = 0;
            for (const auto &n : get_neighbors_8(ok))
            {
                if (obstacle_set.find(n) != obstacle_set.end())
                    ++obstacle_neighbors;
            }
            // Remove if isolated (< 2 neighbors)
            if (obstacle_neighbors < 2)
                to_remove.push_back(ok);
        }

        for (const auto &k : to_remove)
            remove_obstacle(k);
    }
}

void GridESDF::fill_obstacle_gaps()
{
    // Fill 1-cell gaps between obstacles (bridge close obstacles)

    std::unordered_set<Key, boost::hash<Key>> obstacle_set;
    for (const auto &[k, cell] : obstacles_)
    {
        if (cell.tile != nullptr)
            obstacle_set.insert(k);
    }

    std::vector<Key> to_add;
    const int ts = params_.tile_size;

    // Check for horizontal gaps
    for (const auto &ok : obstacle_set)
    {
        // Check right neighbor gap
        Key right1 = {ok.first + ts, ok.second};
        Key right2 = {ok.first + 2*ts, ok.second};
        if (obstacle_set.find(right1) == obstacle_set.end() &&
            obstacle_set.find(right2) != obstacle_set.end())
        {
            to_add.push_back(right1);
        }

        // Check bottom neighbor gap
        Key down1 = {ok.first, ok.second + ts};
        Key down2 = {ok.first, ok.second + 2*ts};
        if (obstacle_set.find(down1) == obstacle_set.end() &&
            obstacle_set.find(down2) != obstacle_set.end())
        {
            to_add.push_back(down1);
        }
    }

    // Add the gap fillers
    for (const auto &k : to_add)
    {
        auto it = obstacles_.find(k);
        if (it == obstacles_.end())
        {
            ObstacleCell cell;
            cell.log_odds = params_.log_odds_max;
            cell.hits = params_.min_hits_to_confirm + 1;
            cell.last_seen = 0;

            if (scene_)
            {
                const float tsf = static_cast<float>(ts);
                cell.tile = scene_->addRect(-tsf/2, -tsf/2, tsf, tsf,
                                            QPen(Qt::NoPen),
                                            QBrush(QColor(params_.obstacle_color)));
                cell.tile->setPos(k.first + tsf/2, k.second + tsf/2);
                cell.tile->setZValue(1);
            }
            obstacles_[k] = cell;
        }
    }

    if (!to_add.empty())
        visualization_dirty_ = true;
}
