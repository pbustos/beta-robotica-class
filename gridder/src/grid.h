/* Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

/* Grid class implements a 2D occupancy grid map. It is bases on a hashmap (unorderedmap) of cells of type T
 * It is used to represent the environment as seen from the robot's coordinate frame, regenerating at each iteration of the percepcion cycle.
 * It has methods to edit the cells and to compute sets of optimal paths using the Dijkstra algorithm and the Yen algorithm.
*/

#ifndef GRID_H
#define GRID_H

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <fstream>
#include <limits>
#include <tuple>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <chrono>
#include <unistd.h>
#include <QtCore>
#include <Eigen/Dense>
#include <QVector2D>
#include <QColor>

class Grid
{
        using Myclock = std::chrono::system_clock;
        using Msec = std::chrono::duration<double, std::milli>;
        using Seconds = std::chrono::seconds;
        using Dimensions = QRectF;

public:
        struct T
        {
            std::uint32_t id;
            bool free = true;
            bool visited = false;
            float cost = 1;
            float hits = 0;
            float misses = 0;
            QGraphicsRectItem *tile;
            // methods to save and read values to disk
            void save(std::ostream &os) const
            { os << free << " " << visited; };
            void read(std::istream &is)
            { is >> free >> visited; };
        };

        using Key = std::pair<int, int>;
        using FMap = std::unordered_map<Key, T, boost::hash<Key>>;

        void initialize(QRectF dim_,
                        int tile_size,
                        QGraphicsScene *scene,
                        QPointF grid_center = QPointF(0,0),
                        float grid_angle = 0.f);

        void check_and_resize(const std::vector<Eigen::Vector3f> &points);
        void resize_grid(QRectF new_dim);

        // paths
        std::vector<Eigen::Vector2f> compute_path_line_of_sight(const Key &source_key, const Key &target_key, const int distance);
        std::vector<std::vector<Eigen::Vector2f>> compute_k_paths(const Key &source_,
                                                                  const Key &target_,
                                                                  unsigned num_paths,
                                                                  float threshold_dist,
                                                                  bool try_closest_free_point,
                                                                  bool target_is_human);

        // map maintainance
        void update_map( const std::vector<Eigen::Vector3f> &points,
                         const Eigen::Vector2f &robot_in_grid,
                         float max_laser_range);
        void update_costs(float robot_semi_width, bool color_all_cells=true);
        inline std::tuple<bool, T &> get_cell(const Key &k);
        Key point_to_key(long int x, long int z) const;
        Key point_to_key(const QPointF &p) const;
        Key point_to_key(const Eigen::Vector2f &p) const;
        size_t size() const { return fmap.size(); };
        Eigen::Vector2f point_to_grid(const Eigen::Vector2f &p) const;
        void clear();   // sets all cells to initial values
        void reset();   // removes all cells from memory
        QRectF get_dim() const { return dim; };

        // iterators
        typename FMap::iterator begin()
        { return fmap.begin(); };
        typename FMap::iterator end()
        { return fmap.end(); };
        typename FMap::const_iterator begin() const
        { return fmap.begin(); };
        typename FMap::const_iterator end() const
        { return fmap.begin(); };
        // TODO: add line, rectangle, circle, espiral, submap, iterators to access regions of the grid
        // as in https://github.com/ANYbotics/grid_map/blob/master/grid_map_core/src/iterators/GridMapIterator.cpp

        // cell access
        inline void insert(const Key &key, const T &value);
        void set_free(const Key &k);
        void set_free(const QPointF &p);
        void set_free(long int x, long int y);
        void set_free(float xf, float yf);
        inline bool is_free(const Key &k);
        inline bool is_free(const Eigen::Vector2f &p);
        inline bool is_occupied(const Eigen::Vector2f &p);
        void set_visited(const Key &k, bool visited);
        bool is_visited(const Key &k);
        void set_all_to_not_visited();
        void set_all_to_free();
        void set_occupied(const Key &k);
        void set_occupied(long int x, long int y);
        void set_occupied(const QPointF &p);
        void set_cost(const Key &k, float cost);
        float get_cost(const Eigen::Vector2f &p);

        // cell probabilistic update
        inline void add_miss(const Eigen::Vector2f &p);
        inline void add_hit(const Eigen::Vector2f &p);
        double log_odds(double prob);
        double retrieve_p(double l);
        size_t count_total() const;
        int count_total_visited() const;

        // constrained grid access
        void mark_area_in_grid_as(const QPolygonF &poly, bool free);   // if true area becomes free
        void modify_cost_in_grid(const QPolygonF &poly, float cost);
        std::optional<QPointF> closest_obstacle(const QPointF &p);
        std::optional<QPointF> closest_free(const QPointF &p);
        std::optional<QPointF> closest_free_4x4(const QPointF &p);
        std::tuple<bool, QVector2D> vector_to_closest_obstacle(QPointF center);
        std::vector<std::pair<Key, T&>> neighboors(const Key &k, const std::vector<int> &xincs, const std::vector<int> &zincs, bool all = false);
        std::vector<std::pair<Key, T&>> neighboors_8(const Key &k, bool all = false);
        std::vector<std::pair<Key, T&>> neighboors_16(const Key &k, bool all = false);

        // path related
        bool is_path_blocked(const std::vector<Eigen::Vector2f> &path);
        bool is_line_of_sigth_to_target_free(const Key &source, const Key &target, float robot_semi_width);
        std::tuple< bool, ::std::string, Grid::Key, Grid::Key>
        validate_source_target(const Eigen::Vector2f &source_, float s_radius,
                               const Eigen::Vector2f &target_, float t_radius);
        void restore_source_target(const Key &source_key, const Key &target_key);


        //Submap operations
        std::vector<std::tuple<Grid::Key,Grid::T>> copy_submap(const Key &center, float radius);
        void paste_submap(const std::vector<std::tuple<Grid::Key, Grid::T>> &submap);
        void set_submap(const Key &center, float radius, bool setFree);
        void set_submap_free(const Key &center,  float radius);

    private:
        FMap fmap;
        QGraphicsScene *scene;
        double updated=0.0, flipped=0.0;
        std::vector<Key> keys;  // vector of keys to compute closest matches
        Dimensions dim = QRectF();
        QGraphicsRectItem *bounding_box = nullptr;
        QPointF grid_center;
        float grid_angle;
        std::uint32_t last_id = 0;

        float frechet_distance(const std::vector<Eigen::Vector2f> &A, const std::vector<Eigen::Vector2f> &B);
        float max_distance(const std::vector<Eigen::Vector2f> &pathA, const std::vector<Eigen::Vector2f> &pathB);
        std::vector<Eigen::Vector2f> compute_path(const Eigen::Vector2f &source_, const Eigen::Vector2f &target_);
        std::vector<Eigen::Vector2f> compute_path_key(const Key &source_key, const Key &target_key);
        std::vector<Eigen::Vector2f> recover_path(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target);
        inline double heuristicL2(const Key &a, const Key &b) const;
        double heuristicL1(const Key &a, const Key &b) const;
        std::vector<Eigen::Vector2f> decimate_path(const std::vector<Eigen::Vector2f> &path, unsigned int step=2);
        std::optional<QPointF> closestMatching_spiralMove(const QPointF &p, const std::function<bool(std::pair<Grid::Key, Grid::T>)> &pred);
        void set_all_costs(float value);


        struct Params
        {
            int tile_size = 100;
            const float free_cost = 1.f;
            const float unknown_cost = 4.f;
            const float occupied_cost = 100.f;
            const QString free_color = "white";
            const QString unknown_color = "LightGrey";
            const QString occupied_color = "DarkRed";
            const float occupancy_threshold = 0.485;

        };
        Params params;


};
#endif // GRID_H
