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
    // Only consider confirmed obstacles (enough hits and confidence)
    return it->second.hits >= params_.min_hits_to_confirm &&
           it->second.log_odds >= params_.occupancy_threshold;
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

        // Create visual tile only when confidence is high enough
        if (cell.tile == nullptr &&
            cell.hits >= params_.min_hits_to_confirm &&
            cell.log_odds >= params_.occupancy_threshold &&
            scene_)
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
            add_obstacle(k, timestamp);
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
    // Sparse sampling: check every 2-3 cells along the ray
    const float tile_size_f = static_cast<float>(params_.tile_size);
    const float step = tile_size_f * 2.5f;  // Sample every ~2.5 cells

    const Eigen::Vector2f delta = to - from;
    const float length = delta.norm() - tile_size_f;  // Don't clear the hit cell
    if (length <= 0) return;

    const Eigen::Vector2f dir = delta.normalized();

    for (float t = tile_size_f; t < length; t += step)
    {
        Eigen::Vector2f sample = from + dir * t;
        Key k = point_to_key(sample);

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

    // Check cache
    auto it = esdf_cache_.find(k);
    if (it != esdf_cache_.end())
        return it->second.distance;

    // Not in cache - compute on demand by finding nearest obstacle
    // This is O(n) but only happens for uncached queries
    float min_dist = std::numeric_limits<float>::max();
    const Eigen::Vector2f p = key_to_point(k);

    for (const auto &[ok, _] : obstacles_)
    {
        const Eigen::Vector2f op = key_to_point(ok);
        const float dist = (p - op).norm();
        if (dist < min_dist)
            min_dist = dist;
    }

    // Cache the result
    esdf_cache_[k] = {min_dist, {0, 0}};  // TODO: track nearest obstacle
    return min_dist;
}

float GridESDF::get_distance(const Eigen::Vector2f &p)
{
    return get_distance(point_to_key(p));
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
                                                     const Eigen::Vector2f &target)
{
    const Key source_key = point_to_key(source);
    const Key target_key = point_to_key(target);
    const float tile_size_f = static_cast<float>(params_.tile_size);

    // A* with ESDF-based cost
    using QElement = std::pair<float, Key>;  // (f_score, key)
    std::priority_queue<QElement, std::vector<QElement>, std::greater<>> open_set;
    std::unordered_map<Key, float, boost::hash<Key>> g_score;
    std::unordered_map<Key, Key, boost::hash<Key>> came_from;

    auto heuristic = [&](const Key &k) -> float {
        return std::hypot(static_cast<float>(k.first - target_key.first),
                         static_cast<float>(k.second - target_key.second));
    };

    g_score[source_key] = 0.0f;
    open_set.push({heuristic(source_key), source_key});

    while (!open_set.empty())
    {
        auto [f, current] = open_set.top();
        open_set.pop();

        if (current == target_key)
        {
            // Reconstruct path
            std::vector<Eigen::Vector2f> path;
            Key k = target_key;
            while (came_from.find(k) != came_from.end())
            {
                path.push_back(key_to_point(k));
                k = came_from[k];
            }
            path.push_back(key_to_point(source_key));
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto &neighbor : get_neighbors_8(current))
        {
            // Skip obstacles
            if (is_obstacle(neighbor))
                continue;

            // Diagonal cost
            const bool is_diagonal = (neighbor.first != current.first &&
                                     neighbor.second != current.second);
            const float move_cost = is_diagonal ? tile_size_f * 1.414f : tile_size_f;

            // Add ESDF-based cost (prefer paths away from obstacles)
            const float esdf_cost = get_cost(neighbor);
            const float tentative_g = g_score[current] + move_cost + esdf_cost;

            auto it = g_score.find(neighbor);
            if (it == g_score.end() || tentative_g < it->second)
            {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                open_set.push({tentative_g + heuristic(neighbor), neighbor});
            }
        }
    }

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

    for (float t = 0; t < length; t += step)
    {
        const Eigen::Vector2f p = source + dir * t;
        const float dist = get_distance(p);
        if (dist < robot_radius)
            return false;
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////
// Visualization
//////////////////////////////////////////////////////////////////////////////

void GridESDF::update_visualization(bool show_inflation)
{
    if (!scene_) return;

    // Skip if no changes
    if (!visualization_dirty_ && show_inflation)
        return;

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
        return;
    }

    const float ts = static_cast<float>(params_.tile_size);

    // Draw inflation layers around confirmed obstacles only
    std::unordered_set<Key, boost::hash<Key>> drawn;

    for (const auto &[ok, cell] : obstacles_)
    {
        // Only process confirmed obstacles (those with visual tiles)
        if (cell.tile == nullptr) continue;

        // Layer 1: immediate neighbors (orange)
        for (const auto &n1 : get_neighbors_8(ok))
        {
            if (!is_obstacle(n1) && drawn.find(n1) == drawn.end())
            {
                auto *tile = scene_->addRect(-ts/2, -ts/2, ts, ts,
                                             QPen(Qt::NoPen),
                                             QBrush(QColor(params_.inflation_color_1)));
                tile->setPos(n1.first + ts/2, n1.second + ts/2);
                tile->setZValue(0);
                inflation_tiles_.push_back(tile);
                drawn.insert(n1);

                // Layer 2: second ring (green)
                for (const auto &n2 : get_neighbors_8(n1))
                {
                    if (!is_obstacle(n2) && drawn.find(n2) == drawn.end())
                    {
                        auto *tile2 = scene_->addRect(-ts/2, -ts/2, ts, ts,
                                                      QPen(Qt::NoPen),
                                                      QBrush(QColor(params_.inflation_color_2)));
                        tile2->setPos(n2.first + ts/2, n2.second + ts/2);
                        tile2->setZValue(0);
                        inflation_tiles_.push_back(tile2);
                        drawn.insert(n2);
                    }
                }
            }
        }
    }

    visualization_dirty_ = false;
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
