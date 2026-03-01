#pragma once

#include <vector>
#include <Eigen/Dense>

namespace rc
{
    /**
     * PolygonPathPlanner — Grid-free shortest-path planner inside a simple polygon.
     *
     * Uses a Visibility Graph + Dijkstra over an inner boundary polygon offset
     * inward by robot_radius from the original layout.
     *
     * Workflow:
     *   1. set_polygon(vertices)  → subdivides, builds inner polygon & visibility graph (once)
     *   2. plan(start, goal)      → returns waypoint sequence (shortest path)
     */
    class PolygonPathPlanner
    {
        public:
            struct Params
            {
                float robot_radius = 0.4f;          // inward offset from walls (m)
                float max_edge_len = 1.0f;           // max edge length before subdivision (m)
                float max_path_length = 50.0f;       // reject paths longer than this (m)
            };

            PolygonPathPlanner() = default;

            /// Set or update the room polygon (room frame). Rebuilds all internal structures.
            void set_polygon(const std::vector<Eigen::Vector2f>& vertices);

            /// Plan shortest path from start to goal (both in room frame).
            /// Returns ordered waypoints including start and goal. Empty if no path.
            std::vector<Eigen::Vector2f> plan(const Eigen::Vector2f& start,
                                               const Eigen::Vector2f& goal) const;

            /// Check if a point is inside the original polygon.
            bool is_inside(const Eigen::Vector2f& point) const;

            /// Check if the straight segment [a, b] stays inside the inner polygon boundary.
            bool is_visible(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const;

            /// Get all navigation nodes (for debug visualization).
            const std::vector<Eigen::Vector2f>& get_navigable_polygon() const { return shrunk_polygon_; }
            /// Get the subdivided polygon vertices (for debug visualization).
            const std::vector<Eigen::Vector2f>& get_original_polygon() const { return subdivided_polygon_; }
            /// Get the inner boundary polygon (1:1 with subdivided vertices).
            const std::vector<Eigen::Vector2f>& get_inner_polygon() const { return inner_polygon_; }

            /// True after a successful set_polygon() call.
            bool is_ready() const { return !inner_polygon_.empty(); }

            Params params;

        private:
            // ---- geometry ----
            std::vector<Eigen::Vector2f> polygon_;             // original layout
            std::vector<Eigen::Vector2f> subdivided_polygon_;  // polygon_ with extra points on long edges
            std::vector<Eigen::Vector2f> inner_polygon_;       // inner boundary (1:1 with subdivided_polygon_)
            std::vector<Eigen::Vector2f> shrunk_polygon_;      // all nav nodes (= inner_polygon_)

            // ---- visibility graph ----
            struct Edge { int to; float cost; };
            std::vector<std::vector<Edge>> adjacency_;

            void build_visibility_graph();

            // ---- static helpers ----
            static std::vector<Eigen::Vector2f> subdivide_polygon(
                const std::vector<Eigen::Vector2f>& poly, float max_edge_len);

            static std::vector<Eigen::Vector2f> offset_polygon_inward(
                const std::vector<Eigen::Vector2f>& poly, float offset);

            static bool point_in_polygon(const Eigen::Vector2f& p,
                                          const std::vector<Eigen::Vector2f>& poly);

            static bool segments_intersect_proper(const Eigen::Vector2f& a1, const Eigen::Vector2f& a2,
                                                   const Eigen::Vector2f& b1, const Eigen::Vector2f& b2);

            /// Check if segment [a,b] crosses any edge of the original polygon.
            bool segment_crosses_original_boundary(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const;

            /// Check if segment [a,b] crosses any edge of the inner polygon.
            bool segment_crosses_inner_boundary(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const;

            static std::vector<int> dijkstra(const std::vector<std::vector<Edge>>& adj,
                                              int start, int goal);
    };

} // namespace rc

