//
// Created by pbustos on 21/10/24.
//

#include "visibility_graph.h"
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

namespace rc
{
    std::vector<Eigen::Vector2f>
    VisibilityGraph::generate_path(const Eigen::Vector2f &start, const Eigen::Vector2f &goal,
                                   const std::vector<QPolygonF> &obstacles, std::vector<float> boundary)
    {
        // Convert the input data to the format required by the visibility graph algorithm
        if (boundary.size() != 4)
        {
            std::cout << __FUNCTION__ << " Boundary must have 4 elements" << std::endl;
            return {};
        }  //TODO: check also reasonable values
        Boundary boundary_ = {.minX=boundary[0], .minY=boundary[1], .maxX=boundary[2], .maxY=boundary[3]};

        // start and end must be within the boundary
        if (!boundary_.contains({start.x(), start.y()}) || !boundary_.contains({goal.x(), goal.y()}))
        {
            std::cout << __FUNCTION__ << " Start or goal is outside the boundary" << std::endl;
            return {};
        }
        Point start_ = {start.x(), start.y()};
        Point goal_ = {goal.x(), goal.y()};

        std::vector<std::vector<Point>> obs(obstacles.size());
        for (const auto &[i, p]: obstacles | iter::enumerate)
            for (const auto &q: p)
                obs[i].emplace_back(rc::VisibilityGraph::Point{q.x(), q.y()});

        // Build the Voronoi graph
        auto graph = buildVoronoiGraph(start_, goal_, obs, boundary_);

        // Find the shortest path using Dijkstra's algorithm
        auto path = dijkstra(graph, start_, goal_);

        // smooth the path
        auto smoothPath = generate_smooth_path(path, obs, boundary_);

        // Convert the path to the format required by the caller
        std::vector<Eigen::Vector2f> result;
        for (const auto &point: smoothPath)
            result.emplace_back(point.x, point.y);

        return result;
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
    bool VisibilityGraph::is_edge_valid(const VisibilityGraph::Point &p1,
                                        const VisibilityGraph::Point &p2,
                                        const std::vector<std::vector<VisibilityGraph::Point>> &obstacles,
                                        const VisibilityGraph::Boundary &boundary)
    {
        // Check if both points are within the boundary
        if (!boundary.contains(p1) || !boundary.contains(p2))
        {
            return false;
        }

        // Check if the edge intersects any obstacles
        for (const auto &obstacle: obstacles)
        {
            for (size_t i = 0; i < obstacle.size(); ++i)
            {
                Point p3 = obstacle[i];
                Point p4 = obstacle[(i + 1) % obstacle.size()];
                if (edges_intersect(p1, p2, p3, p4))
                {
                    return false;
                }
            }
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
    std::map<VisibilityGraph::Point, std::vector<std::pair<VisibilityGraph::Point, double>>>
    VisibilityGraph::buildVoronoiGraph(const Point &start, const Point &goal,
                                       const std::vector<std::vector<Point>> &obstacles,
                                       const Boundary &boundary)
    {
        // Convert obstacles to a set of points for Delaunay triangulation
        std::vector<Point_2> delaunayPoints;
        for (const auto &obstacle: obstacles)
        {
            for (const auto &vertex: obstacle)
            {
                delaunayPoints.emplace_back(vertex.x, vertex.y);
            }
        }

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
            Point center = {circumcenter.x(), circumcenter.y()};

            // Iterate through neighboring faces to connect circumcenters
            for (int i = 0; i < 3; ++i)
            {
                auto neighbor = face->neighbor(i);
                if (!delaunay.is_infinite(neighbor))
                {
                    auto neighborCircumcenter = delaunay.circumcenter(neighbor);
                    Point neighborCenter = {neighborCircumcenter.x(), neighborCircumcenter.y()};

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
        {
            graph[start] = {};
        }
        if (graph.find(goal) == graph.end())
        {
            graph[goal] = {};
        }

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

        return graph;
    }

    // Generate a smooth path by interpolating between the points
    std::vector<VisibilityGraph::Point> VisibilityGraph::generate_smooth_path(const std::vector<Point> &path,
                                                                              const std::vector<std::vector<Point>> &obstacles,
                                                                              const Boundary &boundary,
                                                                              double spacing)
    {
        if (path.size() < 2) {
            return path; // Not enough points to interpolate
        }

        // Generate evenly spaced points along the path segments
        std::vector<Point> smoothPath;
        smoothPath.push_back(path.front()); // Add the start point

        for (size_t i = 0; i < path.size() - 1; ++i) {
            Point p1 = path[i];
            Point p2 = path[i + 1];
            double segmentLength = distance(p1, p2);
            int numPoints = static_cast<int>(segmentLength / spacing);

            for (int j = 1; j <= numPoints; ++j) {
                double t = j * spacing / segmentLength;
                Point newPoint;
                newPoint.x = (1 - t) * p1.x + t * p2.x;
                newPoint.y = (1 - t) * p1.y + t * p2.y;
                smoothPath.push_back(newPoint);
            }
        }

        smoothPath.push_back(path.back()); // Add the goal point

        // Fit a cubic spline using Eigen's Least Squares to the smooth path
        size_t n = smoothPath.size();
        if (n < 4) {
            return smoothPath; // Not enough points for cubic fitting
        }

        Eigen::VectorXd x(n), y(n);
        for (size_t i = 0; i < n; ++i) {
            x(i) = smoothPath[i].x;
            y(i) = smoothPath[i].y;
        }

        // Set up the Vandermonde matrix for cubic fitting
        Eigen::MatrixXd A(n, 4);
        for (size_t i = 0; i < n; ++i) {
            A(i, 0) = 1;
            A(i, 1) = x(i);
            A(i, 2) = x(i) * x(i);
            A(i, 3) = x(i) * x(i) * x(i);
        }

        // Use the normal equation to solve for the polynomial coefficients
        Eigen::MatrixXd AtA = A.transpose() * A;
        Eigen::VectorXd Aty = A.transpose() * y;

        // Solve using LDLT decomposition for numerical stability
        Eigen::VectorXd coeffs = AtA.ldlt().solve(Aty);
        static Eigen::VectorXd previousCoeffs;
        float alpha = 0.3;

        // Blend the coefficients with the previous coefficients for stability
        if (previousCoeffs.size() == coeffs.size())
            coeffs = alpha * coeffs + (1 - alpha) * previousCoeffs;
        previousCoeffs = coeffs;

        // Generate the final smooth path with blended cubic polynomial fitting
        std::vector<Point> finalSmoothPath;
        double totalLength = 0.0;
        for (size_t i = 1; i < n; ++i)
            totalLength += distance(smoothPath[i - 1], smoothPath[i]);

        double currentLength = 0.0;
        while (currentLength <= totalLength)
        {
            double t = currentLength / totalLength;
            double interpX = (1 - t) * x(0) + t * x(n - 1);

            double interpY = coeffs(0) + coeffs(1) * interpX + coeffs(2) * interpX * interpX + coeffs(3) * interpX * interpX * interpX;
            finalSmoothPath.push_back({interpX, interpY});
            currentLength += spacing;
        }

        finalSmoothPath.push_back(smoothPath.back()); // Add the goal point
        return finalSmoothPath;
    }
}

