//
// Created by pbustos on 21/10/24.
//

#ifndef PERSON_TRACKER_VISIBILITY_GRAPH_H
#define PERSON_TRACKER_VISIBILITY_GRAPH_H

#include <vector>
#include <map>
#include <queue>
#include <cmath>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <QPolygonF>
#include <cppitertools/enumerate.hpp>
#include <QGraphicsScene>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>

namespace rc
{
    /**
     * @class VisibilityGraph
     * @brief Generates a path between two points while avoiding obstacles using a visibility graph approach.
     *
     * This class utilizes Delaunay triangulation and Voronoi diagrams to build the graph.
     */
    class VisibilityGraph
    {
        using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
        using Point_2 = Kernel::Point_2;
        using Delaunay = CGAL::Delaunay_triangulation_2<Kernel>;
        using VD = CGAL::Voronoi_diagram_2<Delaunay, CGAL::Default, CGAL::Default>;

        /**
         * @struct Point
         * @brief Represents a point in 2D space with x and y coordinates.
         *
         * Overloads the < operator for use as a key in std::map.
         */
        struct Point
        {
            double x, y;
            bool operator<(const Point &other) const
            { return (x < other.x) || (x == other.x && y < other.y); }
            bool operator==(const Point &other) const
            { return x == other.x && y == other.y; }
        };

        /**
         * @struct Boundary
         * @brief Represents a rectangular boundary with minimum and maximum x and y coordinates.
         *
         * Provides a method to check if a point is within the boundary.
         */
        struct Boundary
        {
            float minX, minY, maxX, maxY;
            bool contains(const Point& p) const
            { return p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY; }
        };

        using VGraph = std::map<VisibilityGraph::Point, std::vector<std::pair<VisibilityGraph::Point, double>>>;

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
                                                   float step_size, QGraphicsScene *scene);

    private:
        /**
         * @brief Calculates the Euclidean distance between two points.
         *
         * @param p1 The first point.
         * @param p2 The second point.
         * @return The Euclidean distance between p1 and p2.
         */
        double distance(const Point &p1, const Point &p2);

        /**
         * @brief Checks if the line segment between points a and b intersects with the line segment between points c and d.
         *
         * @param a The first point of the first line segment.
         * @param b The second point of the first line segment.
         * @param c The first point of the second line segment.
         * @param d The second point of the second line segment.
         * @return True if the line segments intersect, false otherwise.
         */
        bool edges_intersect(const Point &a, const Point &b, const Point &c, const Point &d);

        /**
         * @brief Determines if the edge between points p1 and p2 is valid.
         *
         * An edge is valid if it does not intersect any obstacles and lies within the boundary.
         *
         * @param p1 The first point of the edge.
         * @param p2 The second point of the edge.
         * @param obstacles A vector of vectors of points representing the obstacles.
         * @param boundary The boundary of the area.
         * @return True if the edge is valid, false otherwise.
         */
        bool is_edge_valid(const VisibilityGraph::Point& p1,
                           const VisibilityGraph::Point& p2,
                           const std::vector<std::vector<VisibilityGraph::Point>>& obstacles,
                           const VisibilityGraph::Boundary& boundary);

        /**
         * @brief Implements Dijkstra's algorithm to find the shortest path from start to goal in the given graph.
         *
         * @param graph The graph represented as a map of points to vectors of pairs of points and distances.
         * @param start The starting point of the path.
         * @param goal The goal point of the path.
         * @return A vector of points representing the shortest path.
         */
        std::vector<Point> dijkstra(const std::map<Point, std::vector<std::pair<Point, double>>> &graph,
                                    const Point &start, const Point &goal);

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
        std::tuple<VGraph, Delaunay> buildVoronoiGraph(const Point &start, const Point &goal,
                                                       const std::vector<std::vector<Point>> &obstacles, const Boundary& boundary);

        /**
         * @brief Generates a new path with evenly spaced points based on the given path and spacing.
         *
         * @param path The original path represented as a vector of points.
         * @param spacing The desired spacing between points in the new path.
         * @return A vector of points representing the new path with evenly spaced points.
         */
        std::vector<Eigen::Vector2f> path_spacer(const std::vector<Eigen::Vector2f> &path, float spacing);

        void draw_delaunay(const Delaunay &delaunay, QGraphicsScene *pScene);

    };
}

#endif //PERSON_TRACKER_VISIBILITY_GRAPH_H