/*
 * GridESDF - Sparse Obstacle Grid with Euclidean Signed Distance Field
 *
 * This implementation follows the VoxBlox approach:
 * - Only stores obstacle cells (sparse representation)
 * - Free space is implicit (if not in map → free)
 * - ESDF computed via Dijkstra multi-source propagation
 * - Much more memory efficient for large environments
 *
 * Copyright 2024
 */

#ifndef GRID_ESDF_H
#define GRID_ESDF_H

#include <unordered_map>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <queue>
#include <vector>
#include <limits>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <Eigen/Dense>

class GridESDF
{
public:
    using Key = std::pair<int, int>;

    // Obstacle cell data (only stored for occupied cells)
    struct ObstacleCell
    {
        float log_odds = 2.0f;      // Confidence (higher = more certain obstacle)
        float hits = 1.0f;          // Number of times observed as obstacle
        uint64_t last_seen = 0;     // Timestamp of last observation
        QGraphicsRectItem *tile = nullptr;  // Visual representation
    };

    // ESDF cache entry (computed on demand or incrementally)
    struct ESDFEntry
    {
        float distance = std::numeric_limits<float>::max();  // Distance to nearest obstacle
        Key nearest_obstacle = {0, 0};  // Which obstacle this distance refers to
    };

    // Sparse maps
    using ObstacleMap = std::unordered_map<Key, ObstacleCell, boost::hash<Key>>;
    using ESDFCache = std::unordered_map<Key, ESDFEntry, boost::hash<Key>>;

    // Configuration
    struct Params
    {
        int tile_size = 100;  // mm
        float log_odds_hit = 1.0f;   // Increased - faster confirmation
        float log_odds_miss = -0.4f;
        float log_odds_min = -2.0f;
        float log_odds_max = 4.0f;
        float occupancy_threshold = 2.0f;  // Reduced - need less confidence to be obstacle
        float free_threshold = -0.5f;      // log_odds < this → free
        float max_esdf_distance = 3000.f;  // mm - max distance to propagate ESDF
        int min_hits_to_confirm = 3;       // Reduced - fewer hits needed for visual
        // Visualization
        QString obstacle_color = "DarkRed";
        QString inflation_color_1 = "Orange";
        QString inflation_color_2 = "LightGreen";
        // Cost parameters
        float obstacle_cost = 100.f;
        float free_cost = 1.f;
        // A* path planning limits
        size_t max_astar_nodes = 50000;      // Maximum nodes to expand before giving up
        float astar_distance_factor = 100.f; // Multiply path distance in cells by this factor
    };

    GridESDF() = default;
    ~GridESDF();

    // Initialization
    void initialize(int tile_size, QGraphicsScene *scene);
    void clear();
    void reset();

    // Map update from LiDAR
    void update(const std::vector<Eigen::Vector2f> &hit_points,
                const Eigen::Vector2f &robot_pos,
                float max_range,
                uint64_t timestamp);

    // Obstacle management
    void add_obstacle(const Key &k, uint64_t timestamp);
    void add_confirmed_obstacle(const Key &k);  // Add obstacle directly as confirmed (for external maps)
    void remove_obstacle(const Key &k);
    bool is_obstacle(const Key &k) const;
    bool is_occupied_for_planning(const Key &k, float robot_radius) const;  // Obstacle + inflation
    bool is_free(const Key &k) const;  // Not obstacle = free (sparse assumption)

    // ESDF queries
    float get_distance(const Key &k);  // Distance to nearest obstacle
    float get_distance(const Eigen::Vector2f &p);
    float get_cost(const Key &k);
    float get_cost(const Eigen::Vector2f &p);

    // Key conversion
    Key point_to_key(const Eigen::Vector2f &p) const;
    Key point_to_key(float x, float y) const;
    Eigen::Vector2f key_to_point(const Key &k) const;

    // Path planning support
    std::vector<Eigen::Vector2f> compute_path(const Eigen::Vector2f &source,
                                               const Eigen::Vector2f &target,
                                               float robot_radius = 0.f,
                                               float safety_factor = 1.f);  // 0=touch walls, 1=prefer center
    bool is_line_of_sight_free(const Eigen::Vector2f &source,
                                const Eigen::Vector2f &target,
                                float robot_radius);

    // Iterators for obstacles only
    typename ObstacleMap::iterator begin() { return obstacles_.begin(); }
    typename ObstacleMap::iterator end() { return obstacles_.end(); }
    typename ObstacleMap::const_iterator begin() const { return obstacles_.begin(); }
    typename ObstacleMap::const_iterator end() const { return obstacles_.end(); }

    // Statistics
    size_t num_obstacles() const { return obstacles_.size(); }
    size_t esdf_cache_size() const { return esdf_cache_.size(); }
    void mark_visualization_dirty() { visualization_dirty_ = true; }  // Call after batch loading

    // Serialization for network transmission
    // Compact cell representation: x, y in mm, cost (0-255 normalized)
    struct MapCell
    {
        int32_t x;          // mm
        int32_t y;          // mm
        uint8_t cost;       // 0=free, 255=obstacle, intermediate=inflation
    };
    struct SerializedMap
    {
        int32_t tile_size;              // mm
        int32_t num_cells;              // number of cells
        std::vector<MapCell> cells;     // only cells with cost > 0
    };
    SerializedMap serialize_map() const;  // Returns compact map for transmission

    // Visualization
    void update_visualization(bool show_inflation = true);

    // Post-processing: morphological operations to smooth obstacles
    void morphological_close(int iterations = 1);  // Fill small holes
    void morphological_open(int iterations = 1);   // Remove isolated points
    void fill_obstacle_gaps();                      // Fill 1-cell gaps between obstacles

    // Accessors
    const Params& params() const { return params_; }
    Params& params() { return params_; }

private:
    Params params_;
    ObstacleMap obstacles_;
    ESDFCache esdf_cache_;
    QGraphicsScene *scene_ = nullptr;

    // ESDF propagation
    void propagate_esdf_from(const std::vector<Key> &seeds);
    void invalidate_esdf_near(const Key &removed_obstacle);

    // Visualization items for inflation layers
    std::vector<QGraphicsRectItem*> inflation_tiles_;
    bool visualization_dirty_ = true;  // Flag to track if visualization needs update
    size_t last_confirmed_obstacles_ = 0;  // Track number of confirmed obstacles

    // Ray tracing for free space (sparse sampling)
    void trace_free_space(const Eigen::Vector2f &from,
                          const Eigen::Vector2f &to,
                          uint64_t timestamp);

    // Neighbor access
    std::vector<Key> get_neighbors_4(const Key &k) const;
    std::vector<Key> get_neighbors_8(const Key &k) const;

    // Count confirmed obstacles (with visual tiles)
    size_t count_confirmed_obstacles() const;
};

#endif // GRID_ESDF_H
