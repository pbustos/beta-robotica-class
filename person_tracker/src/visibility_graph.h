//
// Created by pbustos on 21/10/24.
//

#ifndef PERSON_TRACKER_VISIBILITY_GRAPH_H


#include <vector>
#include <map>
#include <queue>
#include <cmath>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <QPolygonF>
#include <cppitertools/enumerate.hpp>

namespace rc
{
    class VisibilityGraph
    {
        using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
        using Point_2 = Kernel::Point_2;
        using Delaunay = CGAL::Delaunay_triangulation_2<Kernel>;
        using VD = CGAL::Voronoi_diagram_2<Delaunay, CGAL::Default, CGAL::Default>;

        public:
            struct Point
            {
                double x, y;
                // Overloading comparison operators for using Point as a key in std::map
                bool operator<(const Point &other) const
                { return (x < other.x) || (x == other.x && y < other.y); }
            };
            struct Boundary  // Check if a point is within the boundary
            {
                float minX, minY, maxX, maxY;
                bool contains(const Point& p) const{ return p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY;}
            };
            std::vector<Eigen::Vector2f> generate_path(const Eigen::Vector2f &start,
                                                       const Eigen::Vector2f &goal,
                                                       const std::vector<QPolygonF> &obstacles,
                                                       std::vector<float> boundary);

        private:
            double distance(const Point &p1, const Point &p2);
            bool edges_intersect(const Point &a, const Point &b, const Point &c, const Point &d);
            bool is_edge_valid(const VisibilityGraph::Point& p1,
                               const VisibilityGraph::Point& p2,
                               const std::vector<std::vector<VisibilityGraph::Point>>& obstacles,
                               const VisibilityGraph::Boundary& boundary);

            std::vector<Point> dijkstra(const std::map<Point, std::vector<std::pair<Point, double>>> &graph, const Point &start, const Point &goal);

            std::map<VisibilityGraph::Point, std::vector <std::pair < VisibilityGraph::Point, double>>>
            buildVoronoiGraph(const Point &start, const Point &goal, const std::vector<std::vector<Point>> &obstacles, const Boundary& boundary);
            std::vector<VisibilityGraph::Point> generate_smooth_path(const std::vector<Point>& path,
                                                                     const std::vector<std::vector<Point>> &obstacles,
                                                                     const Boundary& boundary,
                                                                     double spacing=300);

    };
}
#endif //PERSON_TRACKER_VISIBILITY_GRAPH_H
