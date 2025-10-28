//
// Created by pbustos on 21/10/24.
//

#ifndef PERSON_TRACKER_VISIBILITY_GRAPH_H
#define PERSON_TRACKER_VISIBILITY_GRAPH_H

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <queue>
// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/Voronoi_diagram_2.h>
// #include <CGAL/Delaunay_triangulation_2.h>
// #include <CGAL/Constrained_Delaunay_triangulation_2.h>
// #include <CGAL/Constrained_triangulation_plus_2.h>
#include <cppitertools/enumerate.hpp>
#include <QGraphicsScene>
#include <CDT.h>
#include <QDebug>


namespace rc
{
    struct Point
    {
        double x=0.0, y=0.0;
        bool operator<(const Point &other) const
        { return (x < other.x) || (x == other.x && y < other.y); }
        bool operator==(const Point &other) const
        { return x == other.x && y == other.y; }
        explicit Point(const QPointF &p) { x = p.x(); y = p.y();}
        Point() = default;
        QPointF to_qpointf() const { return QPointF(x, y);
        };
    };
    // Declaration of the << operator for QDebug and Point
    QDebug operator<<(QDebug dbg, const Point &point);

    /**
     * @class VisibilityGraph
     * @brief Generates a path between two points while avoiding obstacles using a visibility graph approach.
     *
     * This class utilizes Delaunay triangulation and Voronoi diagrams to build the graph.
     */
    class VisibilityGraph
    {
        // using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
        // using Point_2 = Kernel::Point_2;
        // using Delaunay = CGAL::Delaunay_triangulation_2<Kernel>;
        // using VD = CGAL::Voronoi_diagram_2<Delaunay, CGAL::Default, CGAL::Default>;
        //
        // // Define a constrained Delaunay triangulation with support for inserting constraints
        // typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel> CDT;
        // typedef CGAL::Constrained_triangulation_plus_2<CDT> CDTPlus;
        // // To handle Voronoi vertices, you can use the dual or compute circumcenters manually
        // typedef CDTPlus::Face_handle Face_handle;


        // Hash function for Point
        struct PointHash {
            std::size_t operator()(const Point& point) const {
                const auto h1 = std::hash<double>{}(point.x);
                const auto h2 = std::hash<double>{}(point.y);
                return h1 ^ (h2 << 1); // Combine the two hash values
            }
        };

        // Data structures and typedefs for clarity
        using VGraph = std::unordered_map<Point, std::vector<std::pair<Point, double>>, PointHash>;
        using Path = std::vector<Point>;

        // Comparator for the priority queue to create a min-heap
        struct PathCostComparator {
            bool operator()(const std::pair<double, Path>& lhs, const std::pair<double, Path>& rhs) const {
                return lhs.first > rhs.first;
            }
        };

        //using PathQueue = std::priority_queue<std::pair<double, Path>, std::vector<std::pair<double, Path>>, PathCostComparator>;
        using PathQueue = std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<>>;

    public:
        /**
         * @brief Generates a path from start to goal avoiding the given obstacles within the specified boundary.
         *
         * @param start The starting point of the path.
         * @param goal The goal point of the path.
         * @param obstacles A vector of polygons representing the obstacles.
         * @param boundary A vector representing the boundary of the area.
         * @param step_size The spacing between points in the generated path.
         * @return A vector of points representing the generated path.
         */
        std::vector<Eigen::Vector2f> generate_path(const Eigen::Vector2f &start,
                                                   const Eigen::Vector2f &goal,
                                                   const std::vector<QPolygonF> &obstacles,
                                                   const std::vector<QPointF> &boundary,
                                                   float step_size, QGraphicsScene *scene);

    private:
        /**
         * @brief Calculates the Euclidean distance between two points.
         *
         * @param p1 The first point.
         * @param p2 The second point.
         * @return The Euclidean distance between p1 and p2.
         */
        double distance(const QPointF &p1, const QPointF &p2) const;

        Path dijkstra_paths(const VGraph &graph, const QPointF &start, const QPointF &goal, int N=1);

        /**
         * @brief Builds a Voronoi graph based on the Delaunay triangulation of the given obstacles and boundary.
         *
         * Includes the start and goal points in the graph.
         *
         * @param start The starting point of the path.
         * @param goal The goal point of the path.
         * @param obstacles A vector of vectors of points representing the obstacles.
         * @param boundary The boundary of the area.
         * @return The Voronoi graph represented as a map of points to vectors of pairs of points and distances.
         */
        // std::tuple<std::map<Point, std::vector<std::pair<Point, double>>>, CDTPlus>
        // buildVoronoiGraph(const Point &start, const Point &goal, const std::vector<std::vector<Point>> &obstacles,
        //                   const std::vector<QPointF> &boundary);

        CDT::Triangulation<double>
        build_dealunay_graph_CDT(const QPointF &start, const QPointF &goal, const std::vector<std::vector<QPointF>> &obstacles,
                               const std::vector<QPointF> &boundary);

        std::optional<QPointF> computeCircumcenter(const CDT::V2d<double> &A, const CDT::V2d<double> &B, const CDT::V2d<double> &C) const;
        std::optional<QPointF> compute_triangle_center(const CDT::V2d<double> &A, const CDT::V2d<double> &B, const CDT::V2d<double> &C) const;
        std::tuple<std::vector<QPointF>, std::vector<std::pair<QPointF, QPointF>>, VGraph>
            compute_voronoi_from_Delaunay(const CDT::Triangulation<double> &cdt, const Point &start, const Point &goal) const;

        /**
         * @brief Generates a new path with evenly spaced points based on the given path and spacing.
         *
         * @param path The original path represented as a vector of points.
         * @param spacing The desired spacing between points in the new path.
         * @return A vector of points representing the new path with evenly spaced points.
         */
        std::vector<Eigen::Vector2f> path_spacer(const std::vector<Eigen::Vector2f> &path, float spacing);
        bool do_line_segments_intersect(const QPointF &p1, const QPointF &q1, const QPointF &p2, const QPointF &q2) const;
        bool free_path(const QPointF &start, const QPointF &goal, const std::vector<std::vector<QPointF>> &obstacles) const;

        QPolygonF simplifyPolygon(const QPolygonF &polygon, double epsilon);

        //void draw_delaunay(const CDTPlus &delaunay, QGraphicsScene *pScene);
        void draw_delaunay(const CDT::Triangulation<double> &cdt, QGraphicsScene *pScene);
        void draw_graph(const VGraph &graph, QGraphicsScene *pScene, bool erase_only=false);
        void draw_voronoi(const std::vector<QPointF> &vcentres, const std::vector<std::pair<QPointF, QPointF>> &vedges,
                          QGraphicsScene *scene);
    };
}

#endif //PERSON_TRACKER_VISIBILITY_GRAPH_H