#include "grid.h"
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/slice.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/chunked.hpp>
#include <cppitertools/filterfalse.hpp>
#include <cppitertools/count.hpp>
#include <set>
#include <queue>
#include <algorithm>

void Grid::resize_grid(QRectF new_dim)
{
    // Expand to include new_dim
    QRectF final_dim = dim.united(new_dim);

    // Align to tile size
    double left = floor(final_dim.left() / params.tile_size) * params.tile_size;
    double top = floor(final_dim.top() / params.tile_size) * params.tile_size;
    double right = ceil(final_dim.right() / params.tile_size) * params.tile_size;
    double bottom = ceil(final_dim.bottom() / params.tile_size) * params.tile_size;

    final_dim = QRectF(left, top, right - left, bottom - top);

    if (final_dim == dim) return;

    // qInfo() << "Resizing grid from" << dim << "to" << final_dim;

    auto my_color = QColor("White");
    Eigen::Matrix2f matrix;
    matrix << std::cos(this->grid_angle) , -std::sin(this->grid_angle) , std::sin(this->grid_angle) , std::cos(this->grid_angle);

    for(const auto &i: iter::range(final_dim.left(), final_dim.right() + params.tile_size/2.0, static_cast<double>(params.tile_size)))
        for(const auto &j: iter::range(final_dim.top(), final_dim.bottom() + params.tile_size/2.0, static_cast<double>(params.tile_size)))
        {
             // Use tile center convention or corner?
             // In initialize: range(dim.left(), dim.right()+params.tile_size, ...
             // i is the coordinate.

             Key k(static_cast<long>(i), static_cast<long>(j));
             if(fmap.find(k) == fmap.end())
             {
                 T aux;
                 aux.id = last_id++;
                 aux.free = true;
                 aux.visited = false;
                 aux.cost = params.unknown_cost;
                 QGraphicsRectItem *tile = scene->addRect(-params.tile_size / 2.f, -params.tile_size / 2.f, params.tile_size, params.tile_size,
                                                      QPen(my_color), QBrush(my_color));
                 Eigen::Vector2f res = matrix * Eigen::Vector2f(i, j) + Eigen::Vector2f(grid_center.x(), grid_center.y());
                 tile->setPos(res.x(), res.y());
                 tile->setRotation(qRadiansToDegrees(grid_angle));
                 aux.tile = tile;
                 insert(k, aux);
                 keys.emplace_back(i, j);
             }
        }

    dim = final_dim;
    if(bounding_box)
        bounding_box->setRect(dim);
}

void Grid::check_and_resize(const std::vector<Eigen::Vector2f> &points)
{
    if(points.empty()) return;

    // Cache dim values for fast comparison
    const float dim_left = static_cast<float>(dim.left());
    const float dim_right = static_cast<float>(dim.right());
    const float dim_top = static_cast<float>(dim.top());
    const float dim_bottom = static_cast<float>(dim.bottom());

    float min_x = dim_left;
    float max_x = dim_right;
    float min_y = dim_top;
    float max_y = dim_bottom;
    bool needs_resize = false;

    for(const auto &p : points)
    {
        const float px = p.x();
        const float py = p.y();

        if(px < min_x) { min_x = px; needs_resize = true; }
        else if(px > max_x) { max_x = px; needs_resize = true; }

        if(py < min_y) { min_y = py; needs_resize = true; }
        else if(py > max_y) { max_y = py; needs_resize = true; }
    }

    if (needs_resize)
    {
         const float margin = params.tile_size * 5;
         resize_grid(QRectF(QPointF(min_x - margin, min_y - margin),
                           QPointF(max_x + margin, max_y + margin)));
    }
}

void Grid::initialize(  QRectF dim_,
                        int tile_size,
                        QGraphicsScene *scene_,
                        QPointF grid_center_,
                        float grid_angle_)
{
    // static QGraphicsRectItem *bounding_box = nullptr;
    dim = dim_;
    this->grid_center = grid_center_;
    this->grid_angle = grid_angle_;

    qInfo() << "dim" << dim.left() << dim.right() << dim.top() << dim.bottom();
    params.tile_size = tile_size;
    scene = scene_;
    for (const auto &[key, value]: fmap)
    {
        scene->removeItem(value.tile);
        delete value.tile;
    }
    if(this->bounding_box != nullptr) scene->removeItem(this->bounding_box);
    fmap.clear();

    auto my_color = QColor("White");
    last_id=0;
    Eigen::Matrix2f matrix;
    matrix << std::cos(grid_angle) , -std::sin(grid_angle) , std::sin(grid_angle) , std::cos(grid_angle);
    for(const auto &i: iter::range(dim.left(), dim.right()+params.tile_size, static_cast<double>(params.tile_size)))
        for(const auto &j: iter::range(dim.top(), dim.bottom()+params.tile_size, static_cast<double>(params.tile_size)))
        {
            T aux;
            aux.id = last_id++;
            aux.free = true;
            aux.visited = false;
            aux.cost = params.unknown_cost;
            QGraphicsRectItem *tile = scene->addRect(-params.tile_size / 2.f, -params.tile_size / 2.f, params.tile_size, params.tile_size,
                                                     QPen(my_color), QBrush(my_color));
            Eigen::Vector2f res = matrix * Eigen::Vector2f(i, j) + Eigen::Vector2f(grid_center.x(), grid_center.y());
            tile->setPos(res.x(), res.y());
            tile->setRotation(qRadiansToDegrees(grid_angle));
            aux.tile = tile;
            insert(Key(static_cast<long>(i), static_cast<long>(j)), aux);
            keys.emplace_back(i, j);    // list of keys
        }

    // draw bounding box
    // this->bounding_box = scene->addRect(dim, QPen(QColor("Grey"), 40));
    // this->bounding_box->setPos(grid_center);
    // this->bounding_box->setZValue(12);
    // this->bounding_box->setRotation(qRadiansToDegrees(grid_angle));

    qInfo() << __FUNCTION__ <<  "Grid parameters: ";
    qInfo() << "    " << "Dim left corner:" << dim.left();
    qInfo() << "    " <<"Dim top corner:" << dim.top();
    qInfo() << "    " << "Dim width:" << dim.width();
    qInfo() << "    " << "Dim height:" << dim.height();
    qInfo() << "    " << "TILE:" << params.tile_size;
    qInfo() << "    " << "num. rows:" << ceil(dim.width() / params.tile_size) + 1;
    qInfo() << "    " << "num. cols:" << ceil(dim.height() / params.tile_size) + 1;
    qInfo() << "    " << "total elems:" << keys.size() << "(" << (ceil(dim.width() / params.tile_size) + 1) * (ceil(dim.height() / params.tile_size) + 1) << ")";
}
inline void Grid::insert(const Key &key, const T &value)
{
    fmap.insert(std::make_pair(key, value));
}
inline std::tuple<bool, Grid::T&> Grid::get_cell(const Key &k)
{
    static T dummy_cell;  // static dummy for failed lookups
    auto it = fmap.find(k);
    if (it != fmap.end())
        return std::forward_as_tuple(true, it->second);
    else
        return std::forward_as_tuple(false, dummy_cell);
}
Grid::Key Grid::point_to_key(long int x, long int z) const
{
    const int ts = params.tile_size;
    const long left = static_cast<long>(dim.left());
    const long top = static_cast<long>(dim.top());
    long kx = std::lround(static_cast<double>(x - left) / ts);
    long kz = std::lround(static_cast<double>(z - top) / ts);
    return Key{ static_cast<int>(left + kx * ts), static_cast<int>(top + kz * ts)};
};
Grid::Key Grid::point_to_key(const QPointF &p) const
{
    const int ts = params.tile_size;
    const double left = dim.left();
    const double top = dim.top();
    long kx = std::lround((p.x() - left) / ts);
    long kz = std::lround((p.y() - top) / ts);
    return Key{ static_cast<int>(left + kx * ts), static_cast<int>(top + kz * ts)};
};
Grid::Key Grid::point_to_key(const Eigen::Vector2f &p) const
{
    const int ts = params.tile_size;
    const float left = static_cast<float>(dim.left());
    const float top = static_cast<float>(dim.top());
    const int kx = static_cast<int>(std::lround((p.x() - left) / ts));
    const int kz = static_cast<int>(std::lround((p.y() - top) / ts));
    return Key{ static_cast<int>(left) + kx * ts, static_cast<int>(top) + kz * ts};
};
Eigen::Vector2f Grid::point_to_grid(const Eigen::Vector2f &p) const
{
    return Eigen::Vector2f{
        static_cast<float>(std::ceil((p.x() - dim.left()) / params.tile_size)),
        static_cast<float>(std::ceil((p.y() - dim.top()) / params.tile_size))
    };
}

//////////////////////////////// STATUS //////////////////////////////////////////
inline bool Grid::is_free(const Key &k)
{
    const auto &[success, v] = get_cell(k);
    if(success)
        return v.free;
    else
        return false;
}
inline bool Grid::is_free(const Eigen::Vector2f &p)
{
    const auto &[success, v] = get_cell(point_to_key(p));
    if(success)
        return v.free;
    else
        return false;
}
inline bool Grid::is_occupied(const Eigen::Vector2f &p)
{
    const auto &[success, v] = get_cell(point_to_key(static_cast<long int>(p.x()), static_cast<long int>(p.y())));
    if(success)
        return not v.free;
    else
        return true;  // non existing cells are returned as occupied
}
void Grid::set_free(const Key &k)
{
    auto &&[success, v] = get_cell(k);
    if(success)
    {
        v.free = true;
        if(v.tile != nullptr)
            v.tile->setBrush(QBrush(QColor(params.free_color)));
    }
}
void Grid::set_free(long int x, long int y)
{
    auto &&[success, v] = get_cell(point_to_key(x, y));
    if(success)
    {
        v.free = true;
        if (v.tile != nullptr)
            v.tile->setBrush(QBrush(QColor(params.free_color)));
    }
}
void Grid::set_free(const QPointF &p)
{
    set_free(static_cast<long int>(p.x()), static_cast<long int>(p.y()));
}
void Grid::set_free(float xf, float yf)
{
    set_free(static_cast<long int>(xf), static_cast<long int>(yf));
}
void Grid::set_occupied(const Key &k)
{
    auto &&[success, v] = get_cell(k);
    if(success)
    {
        v.free = false;
        if(v.tile != nullptr)
            v.tile->setBrush(QBrush(QColor(params.occupied_color)));
    }
}
void Grid::set_occupied(long int x, long int y)
{
    auto &&[success, v] = get_cell(point_to_key(x, y));
    if(success)
    {
       v.free = false;
       if(v.tile != nullptr)
          v.tile->setBrush(QBrush(QColor(params.occupied_color)));
    }
}
void Grid::set_occupied(const QPointF &p)
{
    set_occupied((long int) p.x(), (long int) p.y());
}
inline void Grid::add_miss(const Eigen::Vector2f &p)
{
    static const QBrush free_brush(QColor(params.free_color));
    auto it = fmap.find(point_to_key(p));
    if(it != fmap.end())
    {
        auto &v = it->second;
        v.misses += 1.f;
        // Log-odds update for free space (miss)
        // Using moderate l_free for stability (like ROS2 obstacle_layer)
        constexpr float l_free = -0.3f;   // log-odds decrease for free observation
        constexpr float l_min = -2.0f;    // minimum log-odds (high confidence free)
        constexpr float l_max = 4.0f;     // maximum log-odds (high confidence occupied)

        v.log_odds += l_free;
        v.log_odds = std::clamp(v.log_odds, l_min, l_max);

        // Convert log-odds to probability
        float prob = 1.0f - 1.0f / (1.0f + std::exp(v.log_odds));

        // Hysteresis: only clear if well below threshold (reduces flickering)
        // Cell must be more confident free before switching state
        if(prob < params.occupancy_threshold_low && v.log_odds < -0.5f)
        {
            v.free = true;
            v.cost = params.free_cost;
            v.tile->setBrush(free_brush);
        }
    }
}
inline void Grid::add_hit(const Eigen::Vector2f &p)
{
    static const QBrush occ_brush(QColor(params.occupied_color));
    auto it = fmap.find(point_to_key(p));
    if(it != fmap.end())
    {
        auto &v = it->second;
        v.hits += 1.f;
        // Log-odds update for occupied space (hit)
        // Higher l_occ for faster obstacle detection but requires multiple hits for stability
        constexpr float l_occ = 1.2f;    // log-odds increase for occupied observation
        constexpr float l_min = -2.0f;   // minimum log-odds (high confidence free)
        constexpr float l_max = 4.0f;    // maximum log-odds (high confidence occupied)

        v.log_odds += l_occ;
        v.log_odds = std::clamp(v.log_odds, l_min, l_max);

        // Convert log-odds to probability
        float prob = 1.0f - 1.0f / (1.0f + std::exp(v.log_odds));

        // Hysteresis: only mark occupied if well above threshold (reduces flickering)
        // Requires at least 2 hits to mark as occupied (log_odds > 2.0 after 2 hits)
        if(prob >= params.occupancy_threshold_high && v.log_odds > 1.5f)
        {
            v.free = false;
            v.cost = params.occupied_cost;
            v.tile->setBrush(occ_brush);
        }
    }
}
double Grid::log_odds(double prob)
{
    // Log odds ratio of p(x):
    //              p(x)
    // l(x) = log ----------
    //              1 - p(x)
    return log(prob / (1 - prob));
}
double Grid::retrieve_p(double l)
{
    // Retrieve p(x) from log odds ratio:
    //                   1
    // p(x) = 1 - ---------------
    //             1 + exp(l(x))

    return 1 - 1 / (1 + exp(l));
}
void Grid::set_visited(const Key &k, bool visited)
{
    auto &&[success, v] = get_cell(k);
    if(success)
    {
        v.visited = visited;
        if(visited)
            v.tile->setBrush(QColor("Orange"));
        else
            v.tile->setBrush(QColor("White"));
    }
}
bool Grid::is_visited(const Key &k)
{
    auto &&[success, v] = get_cell(k);
    if(success)
        return v.visited;
    else
        return false;
}
void Grid::set_cost(const Key &k, float cost)
{
    auto &&[success, v] = get_cell(k);
    if(success)
        v.cost = cost;
}
float Grid::get_cost(const Eigen::Vector2f &p)
{
    auto &&[success, v] = get_cell(point_to_key(static_cast<long int>(p.x()), static_cast<long int>(p.y())));
    if(success)
        return v.cost;
    else
        return -1;
}
void Grid::set_all_costs(float value)
{
    for(auto &[key, cell] : fmap)
        cell.cost = value;
}
size_t Grid::count_total() const
{
    return fmap.size();
}
int Grid::count_total_visited() const
{
    int total = 0;
    for(const auto &[k, v] : fmap)
        if(v.visited)
            total ++;
    return total;
}
void Grid::set_all_to_not_visited()
{
    for(auto &[k,v] : fmap)
        set_visited(k, false);
}
void Grid::set_all_to_free()
{
    for(auto &[k,v] : fmap)
        set_free(k);
}
void Grid::mark_area_in_grid_as(const QPolygonF &poly, bool free)
{
    const qreal step = params.tile_size / 4.f;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2.0, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2.0, box.y() + box.height() + step / 2, step))
        {
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
            {
                if (free)
                    set_free(point_to_key(static_cast<long>(x), static_cast<long>(y)));
                else
                    set_occupied(point_to_key(static_cast<long>(x), static_cast<long>(y)));
            }
        }
}
void Grid::modify_cost_in_grid(const QPolygonF &poly, float cost)
{
    const qreal step = params.tile_size / 4.f;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
                set_cost(point_to_key(static_cast<long>(x), static_cast<long>(y)), cost);
}

////////////////////////////////////// PATH //////////////////////////////////////////////////////////////t_)
std::tuple< bool, ::std::string, Grid::Key, Grid::Key> Grid::validate_source_target(const Eigen::Vector2f &source_,
                                                                                    float s_radius,
                                                                                    const Eigen::Vector2f &target_,
                                                                                    float t_radius)
{
    std::string error_msg = "No error";
    // dim to string
    std::string dim_str = "dim: " + std::to_string(dim.left()) + " " + std::to_string(dim.top()) + " " + std::to_string(dim.width()) + " " + std::to_string(dim.height());

    // Admission rules
    if (not dim.contains(QPointF(target_.x(), target_.y())))
    {
        error_msg = "Target " + std::to_string(target_.x()) + " " + std::to_string(target_.y()) + "Target out of limits " + dim_str + " Returning empty path";
        return std::make_tuple(false, error_msg, Grid::Key() , Grid::Key());
    }
    if (not dim.contains(QPointF(source_.x(), source_.y())))
    {
        error_msg = "Source " + std::to_string(source_.x()) + " " + std::to_string(source_.y()) + "Robot  out of limits " + dim_str + " Returning empty path";
        return std::make_tuple(false, error_msg, Grid::Key() , Grid::Key());
    }

    // Get keys
    Key target_key = point_to_key(target_);
    Key source_key = point_to_key(source_);     // TODO: check if this can come back NULL

    //Free cells around the source of path
    set_submap_free(source_key, s_radius);   // robot_semiwidth
    set_submap_free(target_key, t_radius);   // object_semiwidth

//
    // check if source and target are the same
    if (source_key == target_key)
    {
        error_msg = "Robot already at target. Returning empty path";
        return std::make_tuple(false, error_msg, Grid::Key() , Grid::Key());
    }
    error_msg = "Valid source and target";
    return std::make_tuple(true, error_msg, source_key, target_key);
}
void Grid::restore_source_target(const Grid::Key &source_key, const Grid::Key &target_key)
{
    // TODO: we need to store the previous state of the cells to restore them
}

std::vector<Eigen::Vector2f > Grid::compute_path_key(const Key &source_key, const Key &target_key)
{
    const auto &[succ_src, source_cell] = get_cell(source_key);
    const auto &[succ_trg, target_cell] = get_cell(target_key);
    if(!succ_src || !succ_trg)
    {
        qWarning() << __FUNCTION__ << "Source/target key not found in grid";
        return {};
    }
    if(!source_cell.free || !target_cell.free)
    {
        qWarning() << __FUNCTION__ << "Source/target is occupied";
        return {};
    }

    // Dijkstra algorithm
    const size_t map_size = fmap.size();
    std::vector<float> min_distance(map_size, std::numeric_limits<float>::infinity());
    std::vector<std::pair<std::uint32_t, Key>> previous(map_size, std::make_pair(-1, Key()));

    min_distance[source_cell.id] = 0.f;

    using QItem = std::pair<float, Key>;
    auto cmp = [](const QItem &a, const QItem &b) { return a.first > b.first; };
    std::priority_queue<QItem, std::vector<QItem>, decltype(cmp)> pq(cmp);
    pq.push({0.f, source_key});

    while (!pq.empty())
    {
        auto [dist, where] = pq.top();
        pq.pop();

        auto it = fmap.find(where);
        if(it == fmap.end()) continue;
        const auto& where_cell = it->second;

        // Skip if we've already processed this node with a better distance
        if(dist > min_distance[where_cell.id]) continue;

        if (where == target_key)
        {
            auto p = recover_path(previous, source_key, target_key);
            return decimate_path(p);
        }

        for (auto& [neighbor_key, neighbor_cell] : neighboors_8(where))
        {
            const float tentative_dist = min_distance[where_cell.id] + neighbor_cell.cost;

            if (tentative_dist < min_distance[neighbor_cell.id])
            {
                min_distance[neighbor_cell.id] = tentative_dist;
                previous[neighbor_cell.id] = std::make_pair(where_cell.id, where);
                pq.push({tentative_dist, neighbor_key});
            }
        }
    }
    return {};
};
std::vector<Eigen::Vector2f > Grid::compute_path(const Eigen::Vector2f &source_, const Eigen::Vector2f &target_)
{
    // computes a path from source to target using Dijkstra algorithm

    // Admission rules
    if (not dim.contains(QPointF(target_.x(), target_.y())))
    {
        qDebug() << __FUNCTION__ << "Target " << target_.x() << target_.y() << "Target out of limits " << dim << " Returning empty path";
        return {};
    }
    if (not dim.contains(QPointF(source_.x(), source_.y())))
    {
        qDebug() << __FUNCTION__ << "Source " << source_.x() << source_.y() << "Robot  out of limits " << dim << " Returning empty path";
        return {};
    }
    Key target_key = point_to_key(target_);
    const auto &[succ_trg, target_cell] = get_cell(target_key);
    if(not succ_trg)
    {
        qWarning() << "Could not find target position in Grid. Returning empty path";
        return {};
    }
    else if(not target_cell.free)
    {
        qWarning() << "Target position is occupied in Grid. Returning empty path";
        return {};
    }
    Key source_key = point_to_key(source_);
    const auto &[succ_src, source_cell] = get_cell(source_key);
    if(not succ_src)
    {
        qWarning() << "Could not find source position in Grid. Returning empty path";
        return {};
    }
    else if(not source_cell.free)
    {
        qWarning() << "Source position is occupied in Grid. Returning empty path";
        return {};
    }
    if (source_key == target_key)
    {
        qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
        return {};
    }

    // Dijkstra algorithm
    const size_t map_size = fmap.size();
    std::vector<float> min_distance(map_size, std::numeric_limits<float>::infinity());
    std::vector<std::pair<std::uint32_t, Key>> previous(map_size, std::make_pair(-1, Key()));

    min_distance[source_cell.id] = 0.f;

    using QItem = std::pair<float, Key>;
    auto cmp = [](const QItem &a, const QItem &b) { return a.first > b.first; };
    std::priority_queue<QItem, std::vector<QItem>, decltype(cmp)> pq(cmp);
    pq.push({0.f, source_key});

    while (!pq.empty())
    {
        auto [dist, where] = pq.top();
        pq.pop();

        auto it = fmap.find(where);
        if(it == fmap.end()) continue;
        const auto& where_cell = it->second;

        if(dist > min_distance[where_cell.id]) continue;

        if (where == target_key)
        {
            auto p = recover_path(previous, source_key, target_key);
            return decimate_path(p);
        }

        for (auto& [neighbor_key, neighbor_cell] : neighboors_8(where))
        {
            const float tentative_dist = min_distance[where_cell.id] + neighbor_cell.cost;

            if (tentative_dist < min_distance[neighbor_cell.id])
            {
                min_distance[neighbor_cell.id] = tentative_dist;
                previous[neighbor_cell.id] = std::make_pair(where_cell.id, where);
                pq.push({tentative_dist, neighbor_key});
            }
        }
    }
    qInfo() << __FUNCTION__ << "Path from (" << source_key.first << "," << source_key.second << ") to (" <<  target_.x() << "," << target_.y() << ") not  found. Returning empty path";
    return {};
};
std::vector<std::vector<Eigen::Vector2f>> Grid::compute_k_paths(const Key &source_,
                                                                const Key &target_,
                                                                unsigned int num_paths,
                                                                float threshold,
                                                                bool try_closest_free_point,
                                                                bool target_is_human)
{
    // computes at most k paths that differ in max_distance by at least "threshold".
    // the paths are computed using the Yen's algorithm: https://en.wikipedia.org/wiki/Yen%27s_algorithm
    // starting from an initial path and setting to occupied succesive cells in the path, new paths are computed
    // until k paths are found or the initial path is exhausted

    // get an initial shortest path
    std::vector<Eigen::Vector2f> initial_path = compute_path_key(source_, target_);
    if (initial_path.empty())
    {
        qWarning() << __FUNCTION__ << "Initial path to " << target_.first << target_.second << "not found. Returning empty path";
        return {};
    };

    // initialize vector of paths and aux variables
    std::vector<std::vector<Eigen::Vector2f>> paths_list;
    paths_list.push_back(initial_path);
    auto current_step= initial_path.cbegin();   // source
    Key deleted_key = source_;
    bool has_deleted = false;

    // loop until k paths are found or the initial path is exhausted
    while(paths_list.size() < num_paths  and current_step != initial_path.cend())
    {
        // restore previously cell set to occupied
        if(has_deleted)
            set_free(deleted_key);
        // get next key from path and mark it as occupied in the grid
        if(current_step = std::next(current_step); current_step != initial_path.cend())
        {
            // mark cell as occupied
            deleted_key = point_to_key(*current_step);
            has_deleted = true;
            set_occupied(deleted_key);

            auto path = compute_path_key(source_, target_);
            if(not path.empty())
            {
                // check that the new path is different enough from the previous ones
                if(std::ranges::all_of(paths_list, [&path, threshold, this](const auto &p)
                            { return max_distance(p, path) > threshold;}))
                    paths_list.emplace_back(path);

            }
        }
    }
    if(has_deleted)
        set_free(deleted_key);
    return paths_list;
}
//Method to compute line of sight path from source to target
std::vector<Eigen::Vector2f> Grid::compute_path_line_of_sight(const Key &source_key, const Key &target_key, const int distance)
{
    std::vector<Eigen::Vector2f> los_path;
    Eigen::Vector2f source = Eigen::Vector2f{static_cast<float>(source_key.first), static_cast<float>(source_key.second)};
    Eigen::Vector2f target = Eigen::Vector2f{static_cast<float>(target_key.first), static_cast<float>(target_key.second)};

    // fill path with equally spaced points from the robot to the target at a distance of consts.ROBOT_LENGTH
    int npoints = ceil((target-source).norm() / distance);
    if(npoints > 1)
    {
        Eigen::Vector2f dir = (target-source).normalized();  // direction vector from robot to target
        for (const auto &i: iter::range(npoints))
        {
            Eigen::Vector2f p = source + dir * (distance * i);
            los_path.emplace_back(p);
        }
    }
    else
        los_path.emplace_back(target);

    return los_path;
}
std::vector<std::pair<Grid::Key, Grid::T&>> Grid::neighboors(const Grid::Key &k, const std::vector<int> &xincs,const std::vector<int> &zincs,
                                                            bool all)
{
    std::vector<std::pair<Key, T&>> neigh;
    // list of increments to access the neighboors of a given position
    for (auto &&[itx, itz]: iter::zip(xincs, zincs))
    {
        Key lk{k.first + itx, k.second + itz};
        auto &&[success, p] = get_cell(lk);
        if (not success) continue;

        // // if neighboor in diagonal, cost is sqrt(2). Not clear if it changes anything
        //if (itx != 0 and itz != 0 and (fabs(itx) == fabs(itz)) and p.cost == 1)
        //  p.cost = 1.43;

        if (all)
            neigh.emplace_back(lk, p);
        else // only return free neighboors
        {
            if (p.free)
                neigh.emplace_back(lk, p);
        }
    }
    return neigh;
}
std::vector<std::pair<Grid::Key, Grid::T&>> Grid::neighboors_8(const Grid::Key &k, bool all)
{
    const int I = params.tile_size;
    std::vector<std::pair<Key, T&>> neigh;
    neigh.reserve(8);

    // Unrolled loop for 8 neighbors - avoid vector creation
    static constexpr int xincs[8] = {1, 1, 1, 0, -1, -1, -1, 0};
    static constexpr int zincs[8] = {1, 0, -1, -1, -1, 0, 1, 1};

    for(int i = 0; i < 8; ++i)
    {
        Key lk{k.first + xincs[i] * I, k.second + zincs[i] * I};
        auto it = fmap.find(lk);
        if(it != fmap.end())
        {
            if(all || it->second.free)
                neigh.emplace_back(lk, it->second);
        }
    }
    return neigh;
}
std::vector<std::pair<Grid::Key, Grid::T&>> Grid::neighboors_16(const Grid::Key &k, bool all)
{
    const int I = params.tile_size;
    std::vector<std::pair<Key, T&>> neigh;
    neigh.reserve(16);

    static constexpr int xincs[16] = {0, 1, 2, 2, 2, 2, 2, 1, 0, -1, -2, -2, -2, -2, -2, -1};
    static constexpr int zincs[16] = {2, 2, 2, 1, 0, -1, -2, -2, -2, -2, -2, -1, 0, 1, 2, 2};

    for(int i = 0; i < 16; ++i)
    {
        Key lk{k.first + xincs[i] * I, k.second + zincs[i] * I};
        auto it = fmap.find(lk);
        if(it != fmap.end())
        {
            if(all || it->second.free)
                neigh.emplace_back(lk, it->second);
        }
    }
    return neigh;
}
std::vector<Eigen::Vector2f> Grid::recover_path(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target)
{
    // recovers the path from the "previous" vector
    // we use a list here because we want to add elements at the beginning and it is much faster than vector
    std::list<Eigen::Vector2f> aux;
    Key k = target;
    std::uint32_t u = fmap.at(k).id;
    while (previous[u].first != (std::uint32_t)-1)
    {
        aux.emplace_front(static_cast<float>(k.first), static_cast<float>(k.second));
        u = previous[u].first;
        k = previous[u].second;
    }
    // Ensure source is included at the beginning
    aux.emplace_front(static_cast<float>(source.first), static_cast<float>(source.second));
    std::vector<Eigen::Vector2f> res{ std::begin(aux), std::end(aux) };
    return res;
};
std::vector<Eigen::Vector2f> Grid::decimate_path(const std::vector<Eigen::Vector2f> &path, unsigned int step)
{
    // reduces the size of the path by a factor "step"
    // admission rules
    if(step > path.size()/2 )
        return path;

    std::vector<Eigen::Vector2f> res;
    for(auto &&p : iter::chunked(path,step))
        res.push_back(p[0]);
    return res;
}
inline double Grid::heuristicL2(const Key &a, const Key &b) const
{
    return std::hypot(a.first - b.first, a.second - b.second);
}
inline double Grid::heuristicL1(const Key &a, const Key &b) const
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

/////////////////////////////// COSTS /////////////////////////////////////////////////////////
void Grid::update_costs(float robot_semi_width, bool color_all_cells)
{
    static QBrush free_brush(QColor(params.free_color));
    static QBrush occ_brush(QColor(params.occupied_color));
    static QBrush unknown_brush(QColor(params.unknown_color));
    static QBrush orange_brush(QColor("Orange"));
    static QBrush green_brush(QColor("LightGreen"));

    // === PASS 1: Denoise - Remove isolated occupied cells (like ROS2 DenoiseLayer) ===
    // A cell needs at least 2 occupied neighbors to be considered a valid obstacle
    std::vector<Key> isolated_to_clear;
    for (auto &[k, v]: fmap)
    {
        if(v.free) continue;  // skip free cells

        int occ_neighbors = 0;
        for (const auto &[nk, nv] : neighboors_8(k, true))
        {
            if(!nv.free) occ_neighbors++;
        }
        // If less than 2 occupied neighbors, mark as isolated noise
        if(occ_neighbors < 2)
            isolated_to_clear.push_back(k);
    }
    // Clear isolated cells
    for (const auto &k : isolated_to_clear)
    {
        auto &&[ok, cell] = get_cell(k);
        if(!ok) continue;
        cell.free = true;
        cell.cost = params.free_cost;
        // Don't reset log_odds completely - just reduce it
        cell.log_odds = std::max(cell.log_odds - 0.5f, -2.0f);
    }

    // === PASS 2: Normalize costs and colors for all cells ===
    for (auto &[k, v]: fmap)
    {
        const bool is_unknown = (v.hits == 0.f && v.misses == 0.f);
        if(is_unknown)
        {
            v.cost = params.unknown_cost;
            if(color_all_cells)
                v.tile->setBrush(unknown_brush);
        }
        else if(!v.free)
        {
            v.cost = params.occupied_cost;
            if(color_all_cells)
                v.tile->setBrush(occ_brush);
        }
        else
        {
            v.cost = params.free_cost;
            if(color_all_cells)
                v.tile->setBrush(free_brush);
        }
    }

    // === PASS 3: Grow occupied cells (fill small gaps) ===
    // Only mark as occupied if >=5/8 neighbors are occupied
    std::vector<Key> to_occupy;
    to_occupy.reserve(fmap.size() / 8);

    for (auto &[k, v]: fmap)
    {
        const bool is_unknown = (v.hits == 0.f && v.misses == 0.f);
        if(is_unknown || !v.free)
            continue;

        int occ_count = 0;
        for (const auto &[nk, nv] : neighboors_8(k, true))
        {
            if(!nv.free)
            {
                occ_count++;
                if(occ_count >= 5) break;
            }
        }
        if(occ_count >= 5)
            to_occupy.push_back(k);
    }

    for (const auto &k : to_occupy)
    {
        auto &&[ok, cell] = get_cell(k);
        if(!ok) continue;
        cell.free = false;
        cell.cost = params.occupied_cost;
        if(color_all_cells)
            cell.tile->setBrush(occ_brush);
    }

    // === PASS 4: Inflation layer 1 - Orange (exponential decay like ROS2) ===
    std::vector<Key> to_orange;
    to_orange.reserve(fmap.size() / 8);
    // ROS2 uses: cost = INSCRIBED * exp(-cost_scaling_factor * (distance - inscribed_radius))
    const float half_occ_cost = params.occupied_cost / 2.f;

    for (auto &[k, v]: fmap)
    {
        if(v.free) continue;  // only look at occupied cells

        for (const auto &[nk, nv] : neighboors_8(k, true))
        {
            if(nv.free && nv.cost < half_occ_cost)
                to_orange.push_back(nk);
        }
    }

    for (const auto &k : to_orange)
    {
        auto &&[ok, cell] = get_cell(k);
        if(!ok || !cell.free) continue;
        cell.cost = half_occ_cost;
        if(color_all_cells)
            cell.tile->setBrush(orange_brush);
    }

    // === PASS 5: Inflation layer 2 - Green (quarter weight) ===
    std::vector<Key> to_green;
    to_green.reserve(fmap.size() / 8);
    const float quarter_occ_cost = params.occupied_cost / 4.f;

    for (const auto &k : to_orange)
    {
        for (const auto &[nk, nv] : neighboors_8(k, true))
        {
            if(nv.free && nv.cost < quarter_occ_cost)
                to_green.push_back(nk);
        }
    }

    for (const auto &k : to_green)
    {
        auto &&[ok, cell] = get_cell(k);
        if(!ok || !cell.free) continue;
        cell.cost = quarter_occ_cost;
        if(color_all_cells)
            cell.tile->setBrush(green_brush);
    }

    // === PASS 6: Inflation layer 3 - Second green ring (eighth weight) ===
    std::vector<Key> to_green2;
    to_green2.reserve(fmap.size() / 8);
    const float eighth_occ_cost = params.occupied_cost / 8.f;

    for (const auto &k : to_green)
    {
        for (const auto &[nk, nv] : neighboors_8(k, true))
        {
            if(nv.free && nv.cost < eighth_occ_cost)
                to_green2.push_back(nk);
        }
    }

    for (const auto &k : to_green2)
    {
        auto &&[ok, cell] = get_cell(k);
        if(!ok || !cell.free) continue;
        cell.cost = eighth_occ_cost;
        if(color_all_cells)
            cell.tile->setBrush(green_brush);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// ESDF-based map update (VoxBlox style)
/// Much more efficient than ray casting for large maps
//////////////////////////////////////////////////////////////////////////////////////////////

void Grid::update_map_esdf(const std::vector<Eigen::Vector2f> &points,
                           const Eigen::Affine2f &robot_in_grid,
                           float max_laser_range)
{
    if (points.empty()) return;

    const float tile_size_f = static_cast<float>(params.tile_size);
    const float range_threshold = max_laser_range * 0.95f;  // Ignore max-range returns
    const float step_size = tile_size_f * 2.0f;  // Sample every 2 cells for free space (fast)

    const Eigen::Vector2f robot_pos = robot_in_grid.translation();

    // Track modified cells for ESDF propagation
    std::vector<Key> obstacle_seeds;
    obstacle_seeds.reserve(points.size());

    static QBrush free_brush(QColor(params.free_color));
    static QBrush occ_brush(QColor(params.occupied_color));

    for (const auto &point : points)
    {
        const Eigen::Vector2f delta = point - robot_pos;
        const float range = delta.norm();

        // Skip invalid ranges
        if (range < tile_size_f) continue;

        const bool is_max_range = (range >= range_threshold);

        if (!is_max_range)
        {
            // === Mark obstacle cell (endpoint) ===
            Key hit_key = point_to_key(point);
            auto hit_it = fmap.find(hit_key);
            if (hit_it != fmap.end())
            {
                auto &cell = hit_it->second;
                cell.hits += 1.f;

                // Log-odds update for occupied
                constexpr float l_occ = 1.2f;
                constexpr float l_max = 4.0f;
                cell.log_odds = std::min(cell.log_odds + l_occ, l_max);

                if (cell.log_odds > 1.5f)
                {
                    cell.free = false;
                    cell.esdf = 0.0f;
                    cell.esdf_fixed = true;
                    cell.tile->setBrush(occ_brush);
                    obstacle_seeds.push_back(hit_key);
                }
            }
        }

        // === Mark free cells along ray (sparse sampling for efficiency) ===
        // Instead of Bresenham (visits every cell), we sample at step_size intervals
        const Eigen::Vector2f dir = delta.normalized();
        const float trace_range = is_max_range ? range : (range - tile_size_f);  // Don't clear the hit cell

        for (float t = tile_size_f; t < trace_range; t += step_size)
        {
            Eigen::Vector2f sample = robot_pos + dir * t;
            Key free_key = point_to_key(sample);

            auto free_it = fmap.find(free_key);
            if (free_it == fmap.end()) continue;

            auto &cell = free_it->second;

            // Only update if not already marked as obstacle
            if (cell.free || cell.log_odds < 1.0f)
            {
                cell.misses += 1.f;

                // Log-odds update for free
                constexpr float l_free = -0.3f;
                constexpr float l_min = -2.0f;
                cell.log_odds = std::max(cell.log_odds + l_free, l_min);

                if (cell.log_odds < -0.5f)
                {
                    cell.free = true;
                    cell.esdf_fixed = false;
                    cell.tile->setBrush(free_brush);
                }
            }
        }
    }

    // Remove duplicates
    std::sort(obstacle_seeds.begin(), obstacle_seeds.end());
    obstacle_seeds.erase(std::unique(obstacle_seeds.begin(), obstacle_seeds.end()), obstacle_seeds.end());

    // === PHASE 2: Propagate ESDF from obstacle cells (Dijkstra multi-source) ===
    if (!obstacle_seeds.empty())
    {
        propagate_esdf_lower(obstacle_seeds);
    }
}

/**
 * @brief Propagate ESDF using Dijkstra multi-source algorithm
 *
 * This is the "lower wave" from VoxBlox - propagates distances outward from obstacles.
 * Complexity: O(n log n) where n = number of cells affected
 *
 * @param seeds Cells where ESDF = 0 (obstacle boundaries)
 */
void Grid::propagate_esdf_lower(std::vector<Key> &seeds)
{
    const float tile_size_f = static_cast<float>(params.tile_size);

    // Priority queue: (distance, key)
    using QueueElement = std::pair<float, Key>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;

    // Initialize queue with seeds
    for (const auto &k : seeds)
    {
        auto &&[ok, cell] = get_cell(k);
        if (ok && !cell.free)
        {
            cell.esdf = 0.0f;
            cell.esdf_fixed = true;
            pq.push({0.0f, k});
        }
    }

    // Dijkstra propagation
    while (!pq.empty())
    {
        auto [dist, current] = pq.top();
        pq.pop();

        auto &&[ok, cell] = get_cell(current);
        if (!ok) continue;

        // Skip if we already found a better path
        if (dist > cell.esdf) continue;

        // Propagate to 4-connected neighbors (faster than 8-connected)
        static const std::vector<std::pair<int, int>> neighbors_4 = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

        for (const auto &[dx, dy] : neighbors_4)
        {
            Key neighbor_key = {current.first + dx, current.second + dy};
            auto it = fmap.find(neighbor_key);
            if (it == fmap.end()) continue;

            auto &neighbor = it->second;
            const float new_dist = dist + tile_size_f;

            // Update if we found a shorter path
            if (new_dist < neighbor.esdf && !neighbor.esdf_fixed)
            {
                neighbor.esdf = new_dist;
                pq.push({new_dist, neighbor_key});
            }
        }
    }
}

/**
 * @brief Invalidate ESDF values when obstacles are removed (raise wave)
 *
 * This is the "raise wave" from VoxBlox - invalidates distances that depended
 * on a removed obstacle.
 *
 * @param invalidated Cells whose obstacle was removed
 */
void Grid::propagate_esdf_raise(std::vector<Key> &invalidated)
{
    const float tile_size_f = static_cast<float>(params.tile_size);

    std::queue<Key> raise_queue;
    for (const auto &k : invalidated)
    {
        auto &&[ok, cell] = get_cell(k);
        if (ok)
        {
            cell.esdf = std::numeric_limits<float>::max();
            cell.esdf_fixed = false;
            raise_queue.push(k);
        }
    }

    // BFS to invalidate dependent cells
    while (!raise_queue.empty())
    {
        Key current = raise_queue.front();
        raise_queue.pop();

        auto &&[ok, cell] = get_cell(current);
        if (!ok) continue;

        static const std::vector<std::pair<int, int>> neighbors_4 = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

        for (const auto &[dx, dy] : neighbors_4)
        {
            Key neighbor_key = {current.first + dx, current.second + dy};
            auto it = fmap.find(neighbor_key);
            if (it == fmap.end()) continue;

            auto &neighbor = it->second;

            // If neighbor's ESDF depended on the invalidated cell
            if (!neighbor.esdf_fixed && neighbor.esdf < std::numeric_limits<float>::max())
            {
                // Check if this neighbor could have come from current
                if (std::abs(neighbor.esdf - cell.esdf - tile_size_f) < tile_size_f * 0.1f)
                {
                    neighbor.esdf = std::numeric_limits<float>::max();
                    raise_queue.push(neighbor_key);
                }
            }
        }
    }
}

/**
 * @brief Update costs using pre-computed ESDF for efficient inflation
 *
 * Uses the ESDF to compute cost gradients around obstacles.
 * Much more efficient than iterative neighbor expansion.
 *
 * @param robot_radius Robot radius for collision checking
 * @param color_all_cells Whether to update visual colors
 */
void Grid::update_costs_esdf(float robot_radius, bool color_all_cells)
{
    static QBrush free_brush(QColor(params.free_color));
    static QBrush occ_brush(QColor(params.occupied_color));
    static QBrush unknown_brush(QColor(params.unknown_color));
    static QBrush orange_brush(QColor("Orange"));
    static QBrush green_brush(QColor("LightGreen"));

    const float tile_size_f = static_cast<float>(params.tile_size);

    // Inflation distances (in mm)
    const float inflation_radius_1 = tile_size_f * 1.5f;  // Orange zone
    const float inflation_radius_2 = tile_size_f * 3.0f;  // Green zone

    for (auto &[k, v] : fmap)
    {
        const bool is_unknown = (v.hits == 0.f && v.misses == 0.f);

        if (is_unknown)
        {
            v.cost = params.unknown_cost;
            if (color_all_cells)
                v.tile->setBrush(unknown_brush);
        }
        else if (!v.free || v.esdf <= 0.0f)
        {
            // Obstacle cell
            v.cost = params.occupied_cost;
            if (color_all_cells)
                v.tile->setBrush(occ_brush);
        }
        else if (v.esdf < inflation_radius_1)
        {
            // Close to obstacle - orange zone
            // Exponential decay: cost = occ_cost * exp(-alpha * (esdf - 0))
            const float alpha = 2.0f / inflation_radius_1;
            v.cost = params.occupied_cost * std::exp(-alpha * v.esdf);
            if (color_all_cells)
                v.tile->setBrush(orange_brush);
        }
        else if (v.esdf < inflation_radius_2)
        {
            // Medium distance - green zone
            const float alpha = 1.0f / inflation_radius_2;
            v.cost = params.occupied_cost * 0.25f * std::exp(-alpha * (v.esdf - inflation_radius_1));
            if (color_all_cells)
                v.tile->setBrush(green_brush);
        }
        else
        {
            // Far from obstacle - free
            v.cost = params.free_cost;
            if (color_all_cells)
                v.tile->setBrush(free_brush);
        }
    }
}

void Grid::update_map(const std::vector<Eigen::Vector2f> &points,
                      const Eigen::Affine2f &robot_in_grid,
                      float max_laser_range)
{
    const float tile_size_f = static_cast<float>(params.tile_size);
    const float inv_tile_size = 1.0f / tile_size_f;
    // Margin to detect max-range returns (no obstacle detected)
    // Use 5% of max range as margin to filter max-range returns
    const float range_threshold = max_laser_range * 0.95f;

    for(const auto &point : points)
    {
        const Eigen::Vector2f diff = point - robot_in_grid.translation();
        const float length = diff.norm();

        if(length < tile_size_f) continue;  // Skip very short rays

        const int num_steps = static_cast<int>(length * inv_tile_size);
        if(num_steps <= 0) continue;

        const float inv_steps = 1.0f / static_cast<float>(num_steps);
        const Eigen::Vector2f step_vec = diff * inv_steps;

        // Mark cells along the ray as free (miss) - for ALL rays
        Eigen::Vector2f p = robot_in_grid.translation();
        for(int i = 0; i < num_steps - 1; ++i)
        {
            p += step_vec;
            add_miss(p);
        }

        // Only mark endpoint as occupied if ray hit a real obstacle (not max-range)
        if(length < range_threshold)
        {
            add_hit(point);
        }
        // else: ray reached max_range - endpoint is unknown, don't mark as occupied
    }
}

////////////////////////////// DRAW /////////////////////////////////////////////////////////

void Grid::clear()
{
    static const QBrush unknown_brush(QColor(params.unknown_color));
    const float unknown_cost = params.unknown_cost;

    for (auto &[key, value]: fmap)
    {
        value.tile->setBrush(unknown_brush);
        value.free = true;
        value.hits = 0.f;
        value.misses = 0.f;
        value.cost = unknown_cost;
        value.visited = false;
    }
}
void Grid::reset()
{
    for (auto &[key, value]: fmap)
    {
        scene->removeItem(value.tile);
        delete value.tile;
    }
    keys.clear();
    fmap.clear();
}

////////////////////////////// QUERIES /////////////////////////////////////////////////////////
float Grid::max_distance(const std::vector<Eigen::Vector2f> &pathA, const std::vector<Eigen::Vector2f> &pathB)
{
    // Approximates Frechet distance
    std::vector<float> dists;
    for(auto &&i: iter::range(std::min(pathA.size(), pathB.size())))
        dists.emplace_back((pathA[i] - pathB[i]).norm());
    return std::ranges::max(dists);
}
float Grid::frechet_distance(const std::vector<Eigen::Vector2f> &A, const std::vector<Eigen::Vector2f> &B)
{
    // Frechet distance between to paths
    int n = A.size(), m = B.size();
    Eigen::MatrixXf dp(n, m);
    dp(0, 0) = (A[0] - B[0]).norm();

    // Fill first row and column
    for (int i = 1; i < n; ++i)
    {
        dp(i, 0) = std::max(dp(i - 1, 0), (A[i] - B[0]).norm());
    }
    for (int j = 1; j < m; ++j)
    {
        dp(0, j) = std::max(dp(0, j - 1), (A[0] - B[j]).norm());
    }

    // Fill rest of the table
    for (int i = 1; i < n; ++i)
    {
        for (int j = 1; j < m; ++j)
        {
            float minPrevious = std::min({dp(i - 1, j), dp(i, j - 1), dp(i - 1, j - 1)});
            dp(i, j) = std::max(minPrevious, (A[i] - B[j]).norm());
        }
    }
    return dp(n - 1, m - 1);
}
std::optional<QPointF> Grid::closestMatching_spiralMove(const QPointF &p, const std::function<bool(std::pair<Grid::Key, Grid::T>)> &pred)
{
    if(not dim.adjusted(-500, -500, 500, 500).contains(p))  // TODO: remove this hack
    {
        qInfo() << __FUNCTION__ << "Point " << p.x() << p.y() << "out of limits " << dim;
        return {};
    }

    const auto &[ok, cell] = get_cell(point_to_key(p));

    // if not ok, return empty
    //    if(not ok)
    //        return {};

    // if free, return point
    if(ok and cell.free)
        return p;

    int move_unit = params.tile_size;
    int vi = move_unit;
    int vj = 0;
    int tam_segmento = 1;
    int i = static_cast<int>(p.x()), j = static_cast<int>(p.y());
    int recorrido = 0;

    QPointF ret_point;
    while(true)
    {
        i += vi; j += vj; ++recorrido;
        ret_point.setX(i); ret_point.setY(j);
        Key key = point_to_key(ret_point);
        const auto &[success, v] = get_cell(key);
        if(success and pred(std::make_pair(key, v)))
            return ret_point;
        if (recorrido == tam_segmento)
        {
            recorrido = 0;
            int aux = vi; vi = -vj; vj = aux;
            if (vj == 0)
                ++tam_segmento;
        }
    }
}
std::optional<QPointF> Grid::closest_obstacle(const QPointF &p)
{
    return this->closestMatching_spiralMove(p, [](auto cell){ return not cell.second.free; });
}
std::optional<QPointF> Grid::closest_free(const QPointF &p)
{
    return this->closestMatching_spiralMove(p, [](auto cell){ return cell.second.free; });
}
std::optional<QPointF> Grid::closest_free_4x4(const QPointF &p)
{
    return this->closestMatching_spiralMove(p, [this, p](const auto &cell)
            {
                if (not cell.second.free)
                    return false;
                Key key = point_to_key(QPointF(cell.first.first, cell.first.second));
                std::vector<std::pair<Grid::Key, Grid::T&>> L1 = neighboors_16(key, false);
                return (L1.size() == 16);
            });
}
bool Grid::is_path_blocked(const std::vector<Eigen::Vector2f> &path) // grid coordinates
{
    return std::ranges::any_of(path, [this](const auto &p){ return is_occupied(p);});
}
std::tuple<bool, QVector2D> Grid::vector_to_closest_obstacle(QPointF center)
{
    auto k = point_to_key(center);
    QVector2D closestVector;
    bool obstacleFound = false;

    auto neigh = neighboors_8(k, true);
    float dist = std::numeric_limits<float>::max();
    for (auto n : neigh)
    {
        if (not n.second.free)
        {
            QVector2D vec = QVector2D(QPointF(k.first, k.second)) - QVector2D(QPointF(n.first.first,n.first.second)) ;
            if (vec.length() < dist)
            {
                dist = vec.length();
                closestVector = vec;
            }
            qDebug() << __FUNCTION__ << "Obstacle found";
            obstacleFound = true;
        }
    }

    if (!obstacleFound)
    {
        auto DistNeigh = neighboors_16(k, true);
        for (auto n : DistNeigh)
        {
            if (not n.second.free)
            {
                QVector2D vec = QVector2D(QPointF(k.first, k.second)) - QVector2D(QPointF(n.first.first, n.first.second)) ;
                if (vec.length() < dist)
                {
                    dist = vec.length();
                    closestVector = vec;
                }
                obstacleFound = true;
            }
        }
    }
    return std::make_tuple(obstacleFound,closestVector);
}
bool Grid::is_line_of_sigth_to_target_free(const Key &source, const Key &target, float robot_semi_width)
{

    //check if there is a straight line from source to target that is free
    Eigen::Vector2f source_ = Eigen::Vector2f{static_cast<float>(source.first), static_cast<float>(source.second)};
    Eigen::Vector2f target_ = Eigen::Vector2f{static_cast<float>(target.first), static_cast<float>(target.second)};
    float num_steps = ceil((target_ - source_).norm() / static_cast<float>(params.tile_size));
    Eigen::Vector2f step((target_ - source_) / num_steps);
    bool success = true;
    for (auto &&i: iter::range(num_steps))
           if(not is_free(source_ + (step * i)))
           {
               success = false;
               break;
           }
    return success;
}
void Grid::set_submap_free(const Grid::Key &center, float radius)
{
    // set all cells in a square of side "side" around center to free
    for (auto &&[k, v]: iter::filter([center, radius](auto &v)
               { return std::labs(v.first.first - center.first) <= radius and std::labs(v.first.second - center.second) <= radius; }, fmap))
    {
        v.free = true; v.cost = params.free_cost;
    }
}
//make std vector of tuples with key and T
std::vector<std::tuple<Grid::Key,Grid::T>> Grid::copy_submap(const Grid::Key &center, float radius)
{
    static QBrush free_brush(QColor(params.free_color));
    static QBrush occ_brush(QColor(params.occupied_color));
    static QBrush orange_brush(QColor("Orange"));
    static QBrush yellow_brush(QColor("Yellow"));
    static QBrush gray_brush(QColor("LightGray"));
    static QBrush green_brush(QColor("LightGreen"));
    static QBrush white(QColor("White"));
    static std::vector<std::tuple<float, float, QBrush, std::function<std::vector<std::pair<Grid::Key, Grid::T&>>(Grid*, Grid::Key, bool)>>> wall_ranges
                                                                                                                                                     ={{100, 75, orange_brush, &Grid::neighboors_8},
                                                                                                                                                       {75, 50, yellow_brush, &Grid::neighboors_8},
                                                                                                                                                       {50, 5,  green_brush, &Grid::neighboors_16}};
    static std::vector<std::tuple<float, float, QBrush, std::function<std::vector<std::pair<Grid::Key, Grid::T&>>(Grid*, Grid::Key, bool)>>> wall_ranges_no_color
                                                                                                                                                     ={{100, 75, white, &Grid::neighboors_8},
                                                                                                                                                       {75, 50, white, &Grid::neighboors_8},
                                                                                                                                                       {50, 5,  white, &Grid::neighboors_16}};

    std::vector<std::tuple<Grid::Key,Grid::T>> cells;
    std::vector<std::tuple<Grid::Key,Grid::T>> regrown_cells;
    std::vector<std::tuple<Grid::Key,Grid::T>> submap;

    //Get all cells defined by center key and radius
    for (auto &&[k, v]: iter::filter([center, radius](auto &v)
    { return std::labs(v.first.first - center.first) <= radius and std::labs(v.first.second - center.second) <= radius; }, fmap))
    {
        if (v.cost == params.occupied_cost){
            submap.emplace_back(k, v);
        }
    }
    //Get all occupied cost cells in cells
    for(auto &[upper, lower, brush, neigh] : wall_ranges)
    {
        // get all cells with cost == upper
        for (auto &&[k, v]: iter::filter([upper](auto &v) { return std::get<1>(v).cost == upper; }, submap))
        {
            // get all neighboors of these cells whose cost is lower than upper and are free
            auto neighs = neigh(this, k, false);
            for (auto &[kk, vv]: neighs)
            {
                if (vv.cost >= upper || !vv.free)
                    continue;
                const auto &[ok, cell] = get_cell(kk);
                regrown_cells.emplace_back(kk, cell);
            }
        }
    }
    //emplace back all regrown_cells in submap
    for (auto &&[k, v]: regrown_cells)
    {
        submap.emplace_back(k, v);
    }
    return submap;
}
//paste submap into grid
void Grid::paste_submap(const std::vector<std::tuple<Grid::Key,Grid::T>> &submap)
{
    for (auto &&[k, v]: submap)
    {
        fmap[k] = v;
    }
    return;
}
//set submap vector of tuple to occupied or free depending on the value of the bool
void Grid::set_submap(const Key &center, float radius, bool setFree)
{
    // set all cells in a square of side "side" around center to free or occupied using the bool occupied
    for (auto &&[k, v]: iter::filter([center, radius](auto &v)
               { return std::labs(v.first.first - center.first) <= radius and std::labs(v.first.second - center.second) <= radius; }, fmap))
    {
        v.free = setFree;
        v.cost = setFree ?  params.free_cost : params.occupied_cost;
    }
}



// LOS with wide band
//    float num_steps = (target - source).norm() / static_cast<float>(params.tile_size);
//    Eigen::Vector2f step((target - source) / num_steps);
//
//    // compute how many parallel lines we need to cover the robot's width
//    int num_lines_to_side = ceil(robot_semi_width / params.tile_size);
//    bool success = true;
//    for (auto &&i: iter::range(-num_lines_to_side, num_lines_to_side + 1, 1))
//    {
//        Eigen::Vector2f src = Eigen::Vector2f{params.tile_size * i, 0.f};
//        success = success and std::ranges::all_of(iter::range(0.f, num_steps, 1.f), [this, src, step](auto &&i)
//        {
//           bool r = is_free(src + (step * i));
//           return r;
//        });
//    }
//    return success;


// MORRALLA

//    else
//    {
//        //qInfo() << __FUNCTION__ << "Key not found in grid: (" << k.first << k.second << ")";
//        // finds the first element with a key not less than k
//        auto low_x = std::ranges::lower_bound(keys, k, [](const Key &k, const Key &p)
//        { return k.first <= p.first; });
//        if (low_x == keys.end() and not keys.empty())
//            low_x = std::prev(keys.end());
//        std::vector<Key> y_keys;
//        std::copy_if(low_x, std::end(keys), std::back_inserter(y_keys), [low_x](const Key &k)
//        { return k.first == low_x->first; });
//        auto low_y = std::ranges::lower_bound(y_keys, k, [](const Key &k, const Key &p)
//        { return k.second < p.second; });     // z is y
//        if (low_y != y_keys.end())
//        {
//            //qWarning() << __FUNCTION__ << " (2) No key found in grid: Requested (" << k.first << k.second << ") but found ("
//            //           << low_x->x << low_y->z << ")";
//            Key new_key = point_to_key(low_x->first, low_y->second);
//            if (fmap.contains(new_key))
//                return std::forward_as_tuple(true, fmap.at(new_key));
//            else
//                return std::forward_as_tuple(false, T());
//        } else return std::forward_as_tuple(false, T());
//    }


// if target is occupied, try to find the closest free point
//    Eigen::Vector2f target = target_;
//    if(not is_free(target_))
//    {
//        if(auto free = closest_free(QPointF{target_.x(), target_.y()}); free.has_value())
//        {
//            qInfo() << __FUNCTION__ << "Target is occupied. Using closest free point: " << free.value().x() << free.value().y();
//            target = Eigen::Vector2f{free.value().x(), free.value().y()};
//        }
//        else
//        {
//            qInfo() << __FUNCTION__ << "Target is occupied. Could not find closest free point. Returning empty path";
//            return {};
//        }
//    }
