//
// Created by pbustos on 21/10/24.
//

#include "visibility_graph.h"
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cppitertools/range.hpp>
#include <QLineF>

namespace rc
{
    std::vector<Eigen::Vector2f>
    VisibilityGraph::generate_path(const Eigen::Vector2f &start, const Eigen::Vector2f &goal,
                                   const std::vector<QPolygonF> &obstacles,
                                   float step_size, QGraphicsScene *scene)
    {
        static std::vector<Point> ant_path;

        Point start_ = {start.x(), start.y()};
        Point goal_ = {goal.x(), goal.y()};

        std::vector<std::vector<Point>> obs(obstacles.size());
        for (const auto &[i, p]: obstacles | iter::enumerate)
            for (const auto &q: p)
                obs[i].emplace_back(rc::VisibilityGraph::Point{q.x(), q.y()});

        // Build the Voronoi graph
        const auto &[graph, delaunay] = buildVoronoiGraph(start_, goal_, obs, {});

        // Find the shortest path using Dijkstra's algorithm
        auto path = dijkstra(graph, start_, goal_);

        std::vector<Eigen::Vector2f> smooth_path;
        for (const auto &p: path)
            smooth_path.emplace_back(p.x, p.y);
        smooth_path = path_spacer(smooth_path, step_size);

        if(scene != nullptr)
            draw_delaunay(delaunay, scene);

        return smooth_path;
    }

    void VisibilityGraph::draw_delaunay(const Delaunay &delaunay, QGraphicsScene *pScene)
    {
        static std::vector<QGraphicsItem*> items;
        // remove all items drawn in the previous iteration
        for(auto i: items)
        {
            pScene->removeItem(i);
            delete i;
        }
        items.clear();
        QPen pen = QPen(Qt::cyan, 20);
        for (auto face = delaunay.finite_faces_begin(); face != delaunay.finite_faces_end(); ++face)
        {
            for (int i = 0; i < 3; ++i)
            {
                auto neighbor = face->neighbor(i);
                if (!delaunay.is_infinite(neighbor))
                {
                    // get all three vertices of the face
                    auto v0 = face->vertex((i + 1) % 3)->point();
                    auto v1 = face->vertex((i + 2) % 3)->point();
                    auto v2 = face->vertex(i)->point();
                    auto l1 = pScene->addLine(QLineF(CGAL::to_double(v0.x()), CGAL::to_double(v0.y()),
                                                     CGAL::to_double(v1.x()), CGAL::to_double(v1.y())), pen);
                    items.push_back(l1);
                    auto l2 = pScene->addLine(QLineF(CGAL::to_double(v1.x()), CGAL::to_double(v1.y()),
                                                     CGAL::to_double(v2.x()), CGAL::to_double(v2.y())), pen);
                    items.push_back(l2);
                    auto l3 = pScene->addLine(QLineF(CGAL::to_double(v2.x()), CGAL::to_double(v2.y()),
                                                     CGAL::to_double(v0.x()), CGAL::to_double(v0.y())), pen);
                    items.push_back(l3);
                }
            }
        }
    }

    double VisibilityGraph::distance(const Point &p1, const Point &p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    // Check if two edges intersect
    bool VisibilityGraph::edges_intersect(const Point &a, const Point &b, const Point &c, const Point &d)
    {
        auto ccw = [](const Point &p1, const Point &p2, const Point &p3)
        { return (p3.y - p1.y) * (p2.x - p1.x) > (p2.y - p1.y) * (p3.x - p1.x); };
        return (ccw(a, c, d) != ccw(b, c, d)) && (ccw(a, b, c) != ccw(a, b, d));
    }

    // Check if an edge is valid (not intersecting obstacles and within the boundary)
    bool VisibilityGraph::is_edge_valid(const Point &p1,
                                        const Point &p2,
                                        const std::vector<std::vector<Point>> &obstacles,
                                        const Boundary &boundary)
    {
        // Check if both points are within the boundary
//        if (not boundary.contains(p1) or not boundary.contains(p2))
//        {
//            return false;
//        }

        // Check if the edge intersects any obstacles
        for (const auto &obstacle: obstacles)
            for (size_t i = 0; i < obstacle.size(); ++i)
            {
                Point p3 = obstacle[i];
                Point p4 = obstacle[(i + 1) % obstacle.size()];
                if (edges_intersect(p1, p2, p3, p4))
                    return false;
            }
        return true;
    }

    // Dijkstra's algorithm to find the shortest path in the graph
    std::vector<VisibilityGraph::Point>
    VisibilityGraph::dijkstra(const std::map<Point, std::vector<std::pair<Point, double>>> &graph,
                              const Point &start,
                              const Point &goal)
    {
        if (graph.find(start) == graph.end() || graph.find(goal) == graph.end())
            // If start or goal is not in the graph, return an empty path
            return {};

        std::map<Point, double> distances;
        std::map<Point, Point> previous;
        auto compare = [](const std::pair<double, Point> &a, const std::pair<double, Point> &b)
        { return a.first > b.first; };
        std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, decltype(compare)> pq(compare);

        for (const auto &node: graph)
            distances[node.first] = std::numeric_limits<double>::infinity();

        distances[start] = 0;
        pq.emplace(0, start);

        while (!pq.empty())
        {
            Point current = pq.top().second;
            double currentDist = pq.top().first;
            pq.pop();
            if (current.x == goal.x && current.y == goal.y)
            {
                std::vector<Point> path;
                for (Point at = goal; at.x != start.x || at.y != start.y; at = previous[at])
                    path.push_back(at);
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }
            if (currentDist > distances[current])
                continue;
            for (const auto &neighbor: graph.at(current))
            {
                Point next = neighbor.first;
                double weight = neighbor.second;
                double newDist = currentDist + weight;
                if (newDist < distances[next])
                {
                    distances[next] = newDist;
                    previous[next] = current;
                    pq.emplace(newDist, next);
                }
            }
        }
        return {}; // Return an empty path if there is no solution
    }

    // Build the Voronoi graph based on the Delaunay triangulation
    std::tuple<VisibilityGraph::VGraph, VisibilityGraph::Delaunay>
    VisibilityGraph::buildVoronoiGraph(const Point &start, const Point &goal,
                                       const std::vector<std::vector<Point>> &obstacles,
                                       const Boundary &boundary)
    {
        // Convert obstacles to a set of points for Delaunay triangulation
        std::vector<Point_2> delaunayPoints;
        for (const auto &obstacle: obstacles)
            for (const auto &vertex: obstacle)
                delaunayPoints.emplace_back(vertex.x, vertex.y);

        // Add start and goal points
        delaunayPoints.emplace_back(start.x, start.y);
        delaunayPoints.emplace_back(goal.x, goal.y);

        // Construct Delaunay triangulation
        Delaunay delaunay;
        delaunay.insert(delaunayPoints.begin(), delaunayPoints.end());

        // Build the graph based on Voronoi vertices (circumcenters of Delaunay triangles)
        std::map<Point, std::vector<std::pair<Point, double>>> graph;

        for (auto face = delaunay.finite_faces_begin(); face != delaunay.finite_faces_end(); ++face)
        {
            auto circumcenter = delaunay.circumcenter(face);
            Point center = {CGAL::to_double(circumcenter.x()), CGAL::to_double(circumcenter.y())};

            // Iterate through neighboring faces to connect circumcenters
            for (int i = 0; i < 3; ++i)
            {
                auto neighbor = face->neighbor(i);
                if (!delaunay.is_infinite(neighbor))
                {
                    // get all three vertices of the face
//                    auto v0 = face->vertex((i + 1) % 3)->point();
//                    auto v1 = face->vertex((i + 2) % 3)->point();
//                    auto v2 = face->vertex(i)->point();

                    auto neighborCircumcenter = delaunay.circumcenter(neighbor);
                    Point neighborCenter = {CGAL::to_double(neighborCircumcenter.x()), CGAL::to_double(neighborCircumcenter.y())};

                    if (is_edge_valid(center, neighborCenter, obstacles, boundary))
                    {
                        double dist = distance(center, neighborCenter);
                        graph[center].emplace_back(neighborCenter, dist);
                        graph[neighborCenter].emplace_back(center, dist);
                    }
                }
            }
        }

        // Ensure start and goal are added to the graph
        if (graph.find(start) == graph.end())
            graph[start] = {};
        if (graph.find(goal) == graph.end())
            graph[goal] = {};

        // Connect start and goal to their nearest Voronoi nodes
        for (const auto &node: graph)
        {
            Point voronoiNode = node.first;
            double startDist = distance(start, voronoiNode);
            double goalDist = distance(goal, voronoiNode);

            if (is_edge_valid(start, voronoiNode, obstacles, boundary))
            {
                graph[start].emplace_back(voronoiNode, startDist);
                graph[voronoiNode].emplace_back(start, startDist);
            }

            if (is_edge_valid(goal, voronoiNode, obstacles, boundary))
            {
                graph[goal].emplace_back(voronoiNode, goalDist);
                graph[voronoiNode].emplace_back(goal, goalDist);
            }
        }
        return std::make_tuple(graph, delaunay);
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
        for (auto i: iter::range(0UL, num_spaced_points))
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
}