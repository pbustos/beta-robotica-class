#pragma once

#include <vector>
#include <Eigen/Dense>

namespace rc
{
    /**
     * PolygonPathPlanner — Grid-free shortest-path planner inside a simple polygon.
     *
     * Uses Visibility Graph + Dijkstra over the Minkowski-shrunken polygon.
     * Supports future addition of interior obstacle polygons.
     *
     * Workflow:
     *   1. set_polygon(vertices)  → shrinks by robot_radius, builds visibility graph
     *   2. plan(start, goal)      → returns waypoint sequence (shortest path)
     */
    class PolygonPathPlanner
    {
        public:
            struct Params
            {
                float robot_radius = 0.4f;          // Minkowski inward offset (m)
                float waypoint_reached_dist = 0.15f; // Consider waypoint reached within this distance (m)
                float max_path_length = 50.0f;       // Reject paths longer than this (m)
            };

            PolygonPathPlanner() = default;

            /// Set or update the room polygon (room frame). Rebuilds internal structures.
            void set_polygon(const std::vector<Eigen::Vector2f>& vertices);

            /// Plan shortest path from start to goal (both in room frame).
            /// Returns ordered waypoints including start and goal. Empty if no path.
            std::vector<Eigen::Vector2f> plan(const Eigen::Vector2f& start,
                                               const Eigen::Vector2f& goal) const;

            /// Check if a point is inside the navigable (shrunken) polygon.
            bool is_inside(const Eigen::Vector2f& point) const;

            /// Check if a point has robot_radius clearance from all walls.
            bool has_clearance(const Eigen::Vector2f& point) const;

            /// Check if the straight segment [a, b] lies entirely inside the navigable polygon.
            bool is_visible(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const;

            /// Get the shrunken polygon for debug/visualization.
            const std::vector<Eigen::Vector2f>& get_navigable_polygon() const { return shrunk_polygon_; }
            const std::vector<Eigen::Vector2f>& get_original_polygon() const { return subdivided_polygon_; }
            /// Get the inner boundary polygon (one nav node per subdivided vertex, in order).
            const std::vector<Eigen::Vector2f>& get_inner_polygon() const { return inner_polygon_; }

            /// True after a successful set_polygon() call.
            bool is_ready() const { return !shrunk_polygon_.empty(); }

            Params params;

        private:
            // ---- geometry ----
            std::vector<Eigen::Vector2f> polygon_;             // original layout
            std::vector<Eigen::Vector2f> subdivided_polygon_;  // polygon_ with extra points on long edges
            std::vector<Eigen::Vector2f> shrunk_polygon_;      // all nav nodes (vertex + edge midpoint)
            std::vector<Eigen::Vector2f> inner_polygon_;       // inner boundary (1:1 with subdivided_polygon_)

            // ---- visibility graph ----
            struct Edge { int to; float cost; };
            std::vector<std::vector<Edge>> adjacency_;

            void build_visibility_graph();

            // ---- static helpers ----
            /// Subdivide polygon edges so no edge is longer than max_edge_len.
            static std::vector<Eigen::Vector2f> subdivide_polygon(
                const std::vector<Eigen::Vector2f>& poly, float max_edge_len);

            static std::vector<Eigen::Vector2f> offset_polygon_inward(
                const std::vector<Eigen::Vector2f>& poly, float offset);

            static bool point_in_polygon(const Eigen::Vector2f& p,
                                          const std::vector<Eigen::Vector2f>& poly);

            /// Proper segment-segment intersection (excluding shared endpoints).
            static bool segments_intersect_proper(const Eigen::Vector2f& a1, const Eigen::Vector2f& a2,
                                                   const Eigen::Vector2f& b1, const Eigen::Vector2f& b2);

            /// Check if segment [a,b] crosses any edge of the shrunk polygon.
            bool segment_crosses_boundary(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const;

            /// Check if segment [a,b] crosses any edge of the ORIGINAL polygon (the real walls).
            bool segment_crosses_original_boundary(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const;

            /// Dijkstra on an adjacency list. Returns parent indices; -1 = unreachable.
            static std::vector<int> dijkstra(const std::vector<std::vector<Edge>>& adj,
                                              int start, int goal);
        };

} // namespace rc

