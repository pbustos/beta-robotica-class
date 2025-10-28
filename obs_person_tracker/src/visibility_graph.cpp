//
// Created by pbustos on 21/10/24.
//

#include "visibility_graph.h"
#include <algorithm>
#include <QGraphicsItem>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>
#include <QLineF>
#include <QPainter>
#include <cppitertools/sliding_window.hpp>
#include <functional>

namespace rc
{
    // Overload the << operator for QDebug and Point
    QDebug operator<<(QDebug dbg, const Point &point) {
        dbg.nospace() << "(" << point.x << ", " << point.y << ")";
        return dbg.space();
    }

    std::vector<Eigen::Vector2f>
    VisibilityGraph::generate_path(const Eigen::Vector2f &start, const Eigen::Vector2f &goal,
                                   const std::vector<QPolygonF> &obstacles,
                                   const std::vector<QPointF> &boundary,
                                   const float step_size, QGraphicsScene *scene)
    {
        QPointF start_ = {start.x(), start.y()};
        QPointF goal_ = {goal.x(), goal.y()};

        // simplify polygons
        std::vector<QPolygonF> obstacles_;
        for (const auto &poly: obstacles)
            obstacles_.emplace_back(simplifyPolygon(poly, 200));

        // copy the obstacles to a vector of vectors of QPointF
        std::vector<std::vector<QPointF>> obs(obstacles_.size());
        for (const auto &[i, poly]: obstacles_ | iter::enumerate)
            for (const auto &p: poly)
                obs[i].emplace_back(p);

        // check if there is free path from the robot to the goal in a straight line
        if (free_path(start_, goal_, obs))
        {
            draw_graph({}, scene, true);
            return {start, goal};
        }

        // If not FREE PATH build the Delaunay triangulation with CDT lib
        const auto &cdt = build_dealunay_graph_CDT(start_, goal_, obs, boundary);

        // Build de Voronoi diagram
        auto [vcentres, vedges, graph] = compute_voronoi_from_Delaunay(cdt, Point(start_), Point(goal_));

        // Find the shortest path using Dijkstra's algorithm
        const auto paths = dijkstra_paths(graph,start_, goal_, 1);
        if (paths.empty()){ qDebug() << __FUNCTION__ << "Empty path";}

        std::vector<Eigen::Vector2f> smooth_path;
        for (const auto &p: paths)
             smooth_path.emplace_back(p.x, p.y);
        smooth_path = path_spacer(smooth_path, step_size);

        if(scene != nullptr)
        {
            //draw_delaunay(cdt, scene);
            //draw_voronoi(vcentres, vedges, scene);
            draw_graph(graph, scene);
        }
        return smooth_path;
    }

    /////////////////////////////////////////////////////////////////////////////////
    void VisibilityGraph::draw_voronoi(const std::vector<QPointF> &vcentres,
                                       const std::vector<std::pair<QPointF, QPointF>> &vedges,
                                       QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> items;
        for(const auto i: items)
        {
            scene->removeItem(i);
            delete i;
        }
        items.clear();

        const auto pen = QPen(Qt::magenta, 20);
        const auto brush = QBrush(Qt::magenta);
        for (const auto &v: vcentres)
        {
            const auto p = scene->addEllipse(-100, -100, 200, 200, pen);
            p->setPos(v);
            items.push_back(p);
        }

        for (const auto &[p1, p2]: vedges)
        {
            const auto line = scene->addLine(QLineF(p1, p2), pen);
            items.push_back(line);
        }
    }
    void VisibilityGraph::draw_delaunay(const CDT::Triangulation<double> &cdt, QGraphicsScene *pScene)
    {
        static std::vector<QGraphicsItem*> items;
        for(const auto i: items)
        {
            pScene->removeItem(i);
            delete i;
        }
        items.clear();

        const auto pen = QPen(Qt::cyan, 20);

        for (const auto &[vertices, neighbors]: cdt.triangles)
        {
            const CDT::V2d<double> &v1 = cdt.vertices[vertices[0]];
            const CDT::V2d<double> &v2 = cdt.vertices[vertices[1]];
            const CDT::V2d<double> &v3 = cdt.vertices[vertices[2]];

            QGraphicsLineItem* line = pScene->addLine(QLineF(v1.x, v1.y, v2.x, v2.y), pen);
            items.push_back(line);
            QGraphicsLineItem* line2 = pScene->addLine(QLineF(v2.x, v2.y, v3.x, v3.y), pen);
            items.push_back(line2);
            QGraphicsLineItem* line3 = pScene->addLine(QLineF(v3.x, v3.y, v1.x, v1.y), pen);
            items.push_back(line3);
        }
    }
    void VisibilityGraph::draw_graph(const VGraph &graph, QGraphicsScene *pScene, bool erase_only)
    {
        static std::vector<QGraphicsItem*> items;
        for(const auto i: items)
        {
            pScene->removeItem(i);
            delete i;
        }
        items.clear();

        if (not erase_only)
            for (const auto &[p1, neighbors]: graph)
                for (const auto &[p2, weight]: neighbors)
                {
                    const auto line = pScene->addLine(QLineF(p1.to_qpointf(), p2.to_qpointf()), QPen(Qt::green, 20));
                    items.push_back(line);
                }
    }
    double VisibilityGraph::distance(const QPointF &p1, const QPointF &p2) const
    {
        return std::hypot(p1.x() - p2.x(), p1.y() - p2.y());
    }

    // Dijkstra's algorithm to find the shortest path in the graph
    /*std::vector<QPointF>
    VisibilityGraph::dijkstra(const VGraph &graph, const Point &start, const Point &goal)
    {
        // if ( not graph.contains(start) ||  not graph.contains(goal))
        // {
        //     qDebug() << "Start or goal point is not in the graph.";
        //     return {};
        // }

        std::map<Point, double> distances;
        std::map<Point, Point> previous;
        auto compare = [](const std::pair<double, Point>& a, const std::pair<double, Point>& b)
            { return a.first > b.first;}; // Min-heap for priority queue
        std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, decltype(compare)> pq(compare);

        for (const auto &[node, _]: graph)
            distances[Point(node)] = std::numeric_limits<double>::infinity();

        distances[start] = 0;
        pq.emplace(0, start);

        while (not pq.empty())
        {
            Point current = pq.top().second;
            const double currentDist = pq.top().first;
            pq.pop();

            // Use approximate comparison for floating-point values
            if (currentDist > distances[current] + 1e-6)
                continue;

            //if (current.x() == goal.x() and current.y() == goal.y())
            if (qFuzzyCompare(current.x, goal.x) and qFuzzyCompare(current.y, goal.y))
            {
                std::vector<Point> path;
                for (Point at = goal; at.x != start.x || at.y != start.y; at = previous[at])
                    path.push_back(at);
                path.push_back(start);
                std::ranges::reverse(path);
                std::vector<QPointF> path_;
                for (const auto &p: path)
                    path_.emplace_back(p.x, p.y);
                return path_;
            }
            for (const auto &[next, weight]: graph.at(current.to_qpointf()))
                if (double newDist = currentDist + weight; newDist < distances[Point(next)])
                {
                    distances[Point(next)] = newDist;
                    previous[Point(next)] = current;
                    pq.emplace(newDist, next);
                }
        }
        return {}; // Return an empty path if there is no solution
    }
    */

    // Dijkstra algorithm to find the first N optimal paths
    VisibilityGraph::Path VisibilityGraph::dijkstra_paths(const VGraph& graph, const QPointF& start,
                                                          const QPointF& goal, const int N)
    {
        auto pstart = Point(start);
        auto pgoal = Point(goal);

        PathQueue pq;
        std::unordered_map<Point, double, PointHash> costs;
        std::unordered_map<Point, Point, PointHash> predecessors;
        std::unordered_map<Point, bool, PointHash> visited;
        pq.push({0.0, {pstart}});
        costs[pstart] = 0.0;

        while (!pq.empty())
        {
            auto [currentCost, currentNode] = pq.top();
            pq.pop();

            if (visited.contains(currentNode))
                continue;
            visited[currentNode] = true;

            // If the current node is the goal, reconstruct the path
            if (currentNode == pgoal)
            {
                Path path;
                for (Point at = pgoal; at != pstart; at = predecessors[at])
                    path.push_back(at);

                path.push_back(pstart);
                std::ranges::reverse(path);
                return path;
            }

            // Explore neighbors of the current node
            if (graph.contains(currentNode))
            {
                for (const auto& [neighbor, cost] : graph.at(currentNode))
                {
                    const double newCost = currentCost + cost;
                    // If the neighbor hasn't been visited, or we found a cheaper path
                    if (!costs.contains(neighbor) || newCost < costs[neighbor])
                    {
                        costs[neighbor] = newCost;
                        pq.push({newCost, neighbor});
                        predecessors[neighbor] = currentNode;
                    }
                }
            }
        }
        // Return an empty path if no path is found
        return {};
    }

    CDT::Triangulation<double>
    VisibilityGraph::build_dealunay_graph_CDT(const QPointF &start, const QPointF &goal,
                                              const std::vector<std::vector<QPointF>> &obstacles,
                                              const std::vector<QPointF> &boundary)
    {
        // Initialize Constrained Delaunay Triangulation
        auto cdt =  CDT::Triangulation( CDT::VertexInsertionOrder::Enum::Auto,
                                                        CDT::IntersectingConstraintEdges::Enum::TryResolve,
                                                        1e-6);

        std::vector<CDT::V2d<double>> points;
        std::vector<CDT::Edge> edges;

        CDT::VertInd i=0;
        for (const auto &p : boundary)
        {
            points.emplace_back(CDT::V2d<double>(p.x(), p.y()));
            edges.emplace_back(i, (i+1)%boundary.size());
            i++;
        }

        for (const auto &obs : obstacles)
        {
            CDT::VertInd startIdx = i;
            for (const auto &p : obs)
            {
                points.emplace_back(CDT::V2d<double>(p.x(), p.y()));
                edges.emplace_back(i, (i + 1 - startIdx) % obs.size() + startIdx);  // Correct loop for the hole
                i++;
            }
        }

        points.emplace_back(CDT::V2d<double>(start.x(), start.y()));
        points.emplace_back(CDT::V2d<double>(goal.x(), goal.y()));

        cdt.insertVertices(points);
        cdt.insertEdges(edges);
        //cdt.conformToEdges(edges);

        cdt.eraseOuterTrianglesAndHoles();

        // Check if start and goal are inside any obstacle
        // Point_2 start_p(start.x, start.y);
        // Point_2 goal_p(goal.x, goal.y);
        //
        // if (is_point_inside_obstacles(start_p, obstacles) ||
        //     is_point_inside_obstacles(goal_p, obstacles)) {
        //     throw std::invalid_argument("Start or goal point is inside an obstacle.");
        // }

        return cdt;
    }

    /// Function to compute the circumcenter of a triangle
    std::optional<QPointF> VisibilityGraph::compute_triangle_center(const  CDT::V2d<double>& A, const  CDT::V2d<double>& B, const  CDT::V2d<double>& C) const
    {
        const Eigen::Vector2d p1{A.x, A.y};
        const Eigen::Vector2d p2{B.x, B.y};
        const Eigen::Vector2d p3{C.x, C.y};
        Eigen::Vector2d center = (p1 + p2 + p3) / 3.0;
        return QPointF(center.x(), center.y());
    }

    // Example function to generate the Voronoi diagram
    std::tuple<std::vector<QPointF>, std::vector<std::pair<QPointF, QPointF>>, VisibilityGraph::VGraph>
    VisibilityGraph::compute_voronoi_from_Delaunay(const CDT::Triangulation<double> &cdt, const Point &start, const Point &goal) const
    {
        std::vector<QPointF> voronoiVertices;
        std::vector<std::pair<QPointF, QPointF>> voronoiEdges;
        VGraph graph;

        // Iterate over Delaunay triangles to form Voronoi centers and edges
        for (const auto& [vertices, neighbors] : cdt.triangles)
        {
            const auto &v1 = cdt.vertices[vertices[0]];
            const auto &v2 = cdt.vertices[vertices[1]];
            const auto &v3 = cdt.vertices[vertices[2]];
            auto center = compute_triangle_center(v1, v2, v3);
            if (not center) continue;
            voronoiVertices.push_back(center.value());
            for (const auto &neigh : neighbors)  // triangles sharing edges
            {
                 if (neigh > cdt.triangles.size()) continue;
                 auto [vertices, neighbors] = cdt.triangles[neigh];
                 const auto &s1 = cdt.vertices[vertices[0]];
                 const auto &s2 = cdt.vertices[vertices[1]];
                 const auto &s3 = cdt.vertices[vertices[2]];
                 auto s_center = compute_triangle_center(s1,s2, s3);
                 if (not s_center) continue;
                 voronoiEdges.emplace_back(center.value(), s_center.value());
                 auto dist = distance(center.value(), s_center.value());
                 graph[Point(center.value())].emplace_back(s_center.value(), dist);
                 graph[Point(s_center.value())].emplace_back(center.value(), dist);
            }
        }

        // We need to connect the start point to neighbouring centers in Voronoi diagram such that the path is not blocked by obstacles
        const CDT::V2d<double> vstart{start.x, start.y}, vgoal{goal.x, goal.y};
        for (const auto &tri: cdt.triangles)
        {
            for (const auto &[vertices, _] = tri; const auto &v: vertices)
            {
                const auto &v1 = cdt.vertices[tri.vertices[0]];
                const auto &v2 = cdt.vertices[tri.vertices[1]];
                const auto &v3 = cdt.vertices[tri.vertices[2]];
                const auto center = compute_triangle_center(v1, v2, v3);

                if (cdt.vertices[v] == vstart)
                {
                    voronoiEdges.emplace_back(start.to_qpointf(), center.value());
                    graph[start].emplace_back(center.value(), distance(start.to_qpointf(), center.value()));
                    graph[Point(center.value())].emplace_back(start, distance(start.to_qpointf(), center.value()));
                }
                if (cdt.vertices[v] == vgoal)
                {
                    voronoiEdges.emplace_back(goal.to_qpointf(), center.value());
                    graph[goal].emplace_back(center.value(), distance(goal.to_qpointf(), center.value()));
                    graph[Point(center.value())].emplace_back(goal, distance(goal.to_qpointf(), center.value()));
                }
            }
        }
        return {voronoiVertices, voronoiEdges, graph};
    }

    std::vector<Eigen::Vector2f> VisibilityGraph::path_spacer(const std::vector<Eigen::Vector2f> &path, float spacing)
    {
        std::vector<Eigen::Vector2f> spaced_path;
        if (path.size() < 2)
        { return spaced_path; }// Not enough points to interpolate

        // Step 1: Compute cumulative distances between rough points
        std::vector<float> cumulative_distances(path.size(), 0.0);
        for (auto i: iter::range(1UL, path.size()))
            cumulative_distances[i] = cumulative_distances[i - 1] + (path[i - 1] - path[i]).norm();

        // Step 2: Generate N evenly spaced points along the cumulative distance
        double total_distance = cumulative_distances.back();
        auto num_spaced_points = static_cast<size_t>(std::ceil(total_distance / spacing));

        // Interpolate points at intervals of 'spacing' along the path
        double current_distance = 0.0;
        size_t j = 0; // Index for rough_path
        for (auto _: iter::range(0UL, num_spaced_points))
        {
            // Interpolate new point at distance `current_distance`
            while (j < cumulative_distances.size() - 1 and current_distance > cumulative_distances[j + 1])
                ++j; // Move to the next segment
            if (j >= cumulative_distances.size() - 1)
                break;

            // Linear interpolation between rough_path[j] and rough_path[j+1]
            float t = (current_distance - cumulative_distances[j]) /
                      (cumulative_distances[j + 1] - cumulative_distances[j]);
            Eigen::Vector2f new_point = path[j] + t * (path[j + 1] - path[j]);
            spaced_path.push_back(new_point);
            current_distance += spacing; // Move to the next evenly spaced point
        }
        // Ensure the last point is exactly the target
        spaced_path.push_back(path.back());
        return spaced_path;
    }

    // Function to check if two line segments intersect. Returns true if they intersect, false otherwise
    bool VisibilityGraph::do_line_segments_intersect(const QPointF& p1, const QPointF& q1, const QPointF& p2, const QPointF& q2) const
    {
        // Helper to calculate the orientation of ordered triplet (a, b, c)
        auto orientation = [](const QPointF& a, const QPointF& b, const QPointF& c)
        {
            const double val = (b.y() - a.y()) * (c.x() - b.x()) - (b.x() - a.x()) * (c.y() - b.y());
            return (std::abs(val) < 1e-6) ? 0 : (val > 0 ? 1 : 2);
        };

        const int o1 = orientation(p1, q1, p2);
        const int o2 = orientation(p1, q1, q2);
        const int o3 = orientation(p2, q2, p1);
        const int o4 = orientation(p2, q2, q1);

        if (o1 != o2 && o3 != o4) return true;  // General case

        // Special cases where points are collinear and one lies on the other segment
        auto onSegment = [](const QPointF& a, const QPointF& b, const QPointF& c) {
            return b.x() <= std::max(a.x(), c.x()) && b.x() >= std::min(a.x(), c.x()) &&
                   b.y() <= std::max(a.y(), c.y()) && b.y() >= std::min(a.y(), c.y());
        };

        return (o1 == 0 && onSegment(p1, p2, q1)) ||
               (o2 == 0 && onSegment(p1, q2, q1)) ||
               (o3 == 0 && onSegment(p2, p1, q2)) ||
               (o4 == 0 && onSegment(p2, q1, q2));
    }

    // Function to check if there is a free path between two points
    bool VisibilityGraph::free_path(const QPointF &start, const QPointF &goal, const std::vector<std::vector<QPointF>> &obstacles) const
    {
        bool free_path = true;
        for (auto points: obstacles)  // make a copy
        {
            points.emplace_back(points.front());    // to close the polygon
            for (const auto &pp: iter::sliding_window(points, 2))
                if (do_line_segments_intersect(start, goal, pp[0], pp[1]))
                {
                    free_path = false;
                    break;
                }
            if (not free_path) break;
        }
        return free_path;
    }

    // Function to simplify a QPolygonF using the Ramer-Douglas-Peucker algorithm
    QPolygonF VisibilityGraph::simplifyPolygon(const QPolygonF& polygon, const double epsilon)
    {
        if (polygon.size() < 3) return polygon;  // Not enough points to simplify

        // recursive lambda function to simplify the polygon
        std::function<void(const QPolygonF&, int, qsizetype, const double&, QPolygonF&)> rdp =
                    [&](const QPolygonF& poly, int start, int end, double eps, QPolygonF& result) -> void
        {
            double maxDistance = 0.0;
            int index = start;
            for (int i = start + 1; i < end; ++i)
            {
                const double distance = std::abs((poly[end].y() - poly[start].y()) * poly[i].x() -
                                                   (poly[end].x() - poly[start].x()) * poly[i].y() +
                                                    poly[end].x() * poly[start].y() - poly[end].y() * poly[start].x()) /
                                                    std::hypot(poly[end].x() - poly[start].x(), poly[end].y() - poly[start].y());
                if (distance > maxDistance)
                {
                    index = i;
                    maxDistance = distance;
                }
            }

            if (maxDistance > eps)
            {
                rdp(poly, start, index, eps, result);
                result.push_back(poly[index]);
                rdp(poly, index, end, eps, result);
            }
        };

        QPolygonF result;
        result.push_back(polygon.first());
        rdp(polygon, 0, polygon.size() - 1, epsilon, result);
        result.push_back(polygon.last());
        return result;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7777
 // Function to check if a point is inside any obstacle polygon
//     bool VisibilityGraph::is_point_inside_obstacles(const Point_2 &p, const std::vector<std::vector<Point>> &obstacles)
//     {
//     for (const auto &obstacle : obstacles) {
//         if (obstacle.size() < 3) continue; // Not a valid polygon
//
//         CGAL::Polygon_2<Kernel> polygon;
//         for (const auto &vertex : obstacle)
//             polygon.push_back(Point_2(vertex.x, vertex.y));
//
//         if (polygon.bounded_side(p) != CGAL::ON_UNBOUNDED_SIDE)
//             return true; // Point is inside or on the boundary
//     }
//     return false;
// }
//
//     void VisibilityGraph::ensure_correct_orientation(CGAL::Polygon_2<Kernel> &polygon, bool is_hole)
//     {
//         if (is_hole && polygon.orientation() != CGAL::CLOCKWISE)
//             polygon.reverse_orientation();
//         else if (!is_hole && polygon.orientation() != CGAL::COUNTERCLOCKWISE)
//             polygon.reverse_orientation();
//     }

    //Function to build the Voronoi graph using Constrained Delaunay Triangulation
    // std::tuple<std::map<VisibilityGraph::Point, std::vector<std::pair<VisibilityGraph::Point, double>>>, VisibilityGraph::CDTPlus>
    // VisibilityGraph::buildVoronoiGraph(const Point &start, const Point &goal, const std::vector<std::vector<Point>> &obstacles,
    //                                    const std::vector<QPointF> &boundary)
    // {
    //     // Initialize Constrained Delaunay Triangulation
    //     CDTPlus cdt;
    //
    //     // Lambda to insert polygon constraints
    //     auto insert_polygon = [&](const std::vector<Point> &polygon, bool is_hole = false)
    //     {
    //         if (polygon.size() < 3) return; // Not a valid polygon
    //
    //         // Convert to CGAL Polygon_2
    //         CGAL::Polygon_2<Kernel> cgal_polygon;
    //         for (const auto &[x, y] : polygon)
    //             cgal_polygon.push_back(Point_2(x, y));
    //
    //         // Ensure correct orientation
    //         ensure_correct_orientation(cgal_polygon, is_hole);
    //
    //         // Insert edges as constraints
    //         for (size_t i = 0; i < cgal_polygon.size(); ++i)
    //         {
    //             size_t next = (i + 1) % cgal_polygon.size();
    //             cdt.insert_constraint(cgal_polygon[i], cgal_polygon[next]);
    //         }
    //     };
    //
    //     // Insert the outer boundary (assuming it's not a hole)
    //     std::vector<Point> boundary_2;
    //     std::ranges::transform(boundary, std::back_inserter(boundary_2), [](const QPointF &p)
    //     { return Point(p.x(), p.y()); });
    //     insert_polygon(boundary_2, false);
    //
    //     // Insert obstacle polygons as constraints
    //     for (const auto &obstacle : obstacles)
    //         insert_polygon(obstacle, true);
    //
    //     // Check if start and goal are inside any obstacle
    //     Point_2 start_p(start.x, start.y);
    //     Point_2 goal_p(goal.x, goal.y);
    //
    //     if (is_point_inside_obstacles(start_p, obstacles) ||
    //         is_point_inside_obstacles(goal_p, obstacles)) {
    //         throw std::invalid_argument("Start or goal point is inside an obstacle.");
    //     }
    //
    //     // Insert start and goal points
    //     cdt.insert(start_p);
    //     cdt.insert(goal_p);
    //
    //     // Build the Voronoi graph
    //     std::map<Point, std::vector<std::pair<Point, double>>> graph;
    //
    //     for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face) {
    //         // Compute circumcenter
    //         Point_2 cc = cdt.circumcenter(face);
    //
    //         // Skip if circumcenter is inside any obstacle
    //         if (is_point_inside_obstacles(cc, obstacles))
    //             continue;
    //
    //         Point center = {CGAL::to_double(cc.x()), CGAL::to_double(cc.y())};
    //
    //         // Iterate through the three edges of the face
    //         for (int i = 0; i < 3; ++i) {
    //             Face_handle neighbor = face->neighbor(i);
    //             if (cdt.is_infinite(neighbor))
    //                 continue;
    //
    //             // Compute neighbor's circumcenter
    //             Point_2 nc = cdt.circumcenter(neighbor);
    //
    //             // Skip if neighbor's circumcenter is inside any obstacle
    //             if (is_point_inside_obstacles(nc, obstacles))
    //                 continue;
    //
    //             Point neighbor_center = {CGAL::to_double(nc.x()), CGAL::to_double(nc.y())};
    //
    //             // Validate the edge between centers
    //             if (is_edge_valid(center, neighbor_center, obstacles, {})) {
    //                 double dist = distance(center, neighbor_center);
    //                 graph[center].emplace_back(neighbor_center, dist);
    //                 graph[neighbor_center].emplace_back(center, dist);
    //             }
    //         }
    //     }
    //
    //     // Ensure start and goal are present in the graph
    //     Point start_pt = {start.x, start.y};
    //     Point goal_pt = {goal.x, goal.y};
    //     if (graph.find(start_pt) == graph.end())
    //         graph[start_pt] = {};
    //     if (graph.find(goal_pt) == graph.end())
    //         graph[goal_pt] = {};
    //
    //     // Connect start and goal to nearest Voronoi nodes
    //     for (const auto &node : graph)
    //     {
    //         Point voronoiNode = node.first;
    //         double startDist = distance(start_pt, voronoiNode);
    //         double goalDist = distance(goal_pt, voronoiNode);
    //
    //         if (is_edge_valid(start_pt, voronoiNode, obstacles, {}))
    //         {
    //             graph[start_pt].emplace_back(voronoiNode, startDist);
    //             graph[voronoiNode].emplace_back(start_pt, startDist);
    //         }
    //
    //         if (is_edge_valid(goal_pt, voronoiNode, obstacles, {}))
    //         {
    //             graph[goal_pt].emplace_back(voronoiNode, goalDist);
    //             graph[voronoiNode].emplace_back(goal_pt, goalDist);
    //         }
    //     }
    //     return std::make_tuple(graph, cdt);
    // }

  // void VisibilityGraph::draw_delaunay(const CDTPlus &delaunay, QGraphicsScene *pScene)
   // {
   //     static std::vector<QGraphicsItem*> items;
   //     // remove all items drawn in the previous iteration
   //     for(const auto i: items)
   //     {
   //         pScene->removeItem(i);
   //         delete i;
   //     }
   //     items.clear();
   //     const auto pen = QPen(Qt::cyan, 20);
   //     // Extract triangles
   //
   //     // Draw triangles
   //     for (const auto tri: delaunay.finite_face_handles())
   //     {
   //         const auto& p0 = tri->vertex(0)->point();
   //         const auto& p1 = tri->vertex(1)->point();
   //         const auto& p2 = tri->vertex(2)->point();
   //
   //         // Create a line from v1 to v2
   //         QGraphicsLineItem* line = pScene->addLine(QLineF(CGAL::to_double(p0.x()), CGAL::to_double(p0.y()),
   //                                                          CGAL::to_double(p1.x()), CGAL::to_double(p1.y())), pen);
   //         items.push_back(line);
   //         QGraphicsLineItem* line2 = pScene->addLine(QLineF(CGAL::to_double(p1.x()), CGAL::to_double(p1.y()),
   //                                                          CGAL::to_double(p2.x()), CGAL::to_double(p2.y())), pen);
   //         items.push_back(line2);
   //         QGraphicsLineItem* line3 = pScene->addLine(QLineF(CGAL::to_double(p0.x()), CGAL::to_double(p0.y()),
   //                                                          CGAL::to_double(p2.x()), CGAL::to_double(p2.y())), pen);
   //         items.push_back(line3);
   //     }
   // }


// std::optional<QPointF> VisibilityGraph::computeCircumcenter(const CDT::V2d<double>& A, const CDT::V2d<double>& B, const CDT::V2d<double>& C) const
// {
//     // Convert to Eigen vectors for computation
//     Eigen::Vector2d p1{A.x, A.y};
//     Eigen::Vector2d p2{B.x, B.y};
//     Eigen::Vector2d p3{C.x, C.y};
//
//     // Compute midpoints of two sides
//     const Eigen::Vector2d mid1 = (p1 + p2) / 2.0;
//     const Eigen::Vector2d mid2 = (p2 + p3) / 2.0;
//
//     // Compute vectors parallel to sides
//     Eigen::Vector2d vec1 = p2 - p1;
//     Eigen::Vector2d vec2 = p3 - p2;
//
//     // Compute perpendicular vectors (rotate by 90 degrees)
//     const Eigen::Vector2d perp1(-vec1.y(), vec1.x());
//     const Eigen::Vector2d perp2(-vec2.y(), vec2.x());
//
//     // Set up system of equations
//     // mid1 + t*perp1 = mid2 + s*perp2
//     Eigen::Matrix2d SA;
//     SA.col(0) = perp1;
//     SA.col(1) = -perp2;
//
//     const Eigen::Vector2d b = mid2 - mid1;
//
//     // Solve system
//     // Check if matrix is invertible (non-degenerate triangle)
//     if (std::abs(SA.determinant()) < 1e-10)
//         return {};  // Return empty optional for degenerate case
//
//     Eigen::Vector2d ts = SA.colPivHouseholderQr().solve(b);
//     const double t = ts(0);
//
//     // Compute circumcenter
//     Eigen::Vector2d circumcenter = mid1 + t * perp1;
//
//     return QPointF(circumcenter.x(), circumcenter.y());
// }
