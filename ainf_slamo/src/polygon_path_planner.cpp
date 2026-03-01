#include "polygon_path_planner.h"
#include <algorithm>
#include <queue>
#include <cmath>
#include <limits>
#include <iostream>

namespace rc
{

// =====================================================================
// Distance from point to polygon boundary (minimum over all edges)
// =====================================================================
static float dist_to_polygon_boundary(const Eigen::Vector2f& p,
                                       const std::vector<Eigen::Vector2f>& poly)
{
    float best = std::numeric_limits<float>::max();
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i)
    {
        const auto& a = poly[i];
        const auto& b = poly[(i + 1) % n];
        Eigen::Vector2f ab = b - a;
        float t = (p - a).dot(ab) / (ab.squaredNorm() + 1e-10f);
        t = std::clamp(t, 0.f, 1.f);
        float d = (p - (a + t * ab)).norm();
        if (d < best) best = d;
    }
    return best;
}

// =====================================================================
// Point-in-polygon (ray casting)
// =====================================================================
bool PolygonPathPlanner::point_in_polygon(const Eigen::Vector2f& p,
                                           const std::vector<Eigen::Vector2f>& poly)
{
    const int n = static_cast<int>(poly.size());
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        const auto& vi = poly[i];
        const auto& vj = poly[j];
        if (((vi.y() > p.y()) != (vj.y() > p.y())) &&
            (p.x() < (vj.x() - vi.x()) * (p.y() - vi.y()) / (vj.y() - vi.y()) + vi.x()))
            inside = !inside;
    }
    return inside;
}

// =====================================================================
// Segment-segment proper intersection
// =====================================================================
bool PolygonPathPlanner::segments_intersect_proper(
    const Eigen::Vector2f& a1, const Eigen::Vector2f& a2,
    const Eigen::Vector2f& b1, const Eigen::Vector2f& b2)
{
    auto cross2 = [](const Eigen::Vector2f& u, const Eigen::Vector2f& v)
    { return u.x() * v.y() - u.y() * v.x(); };
    const Eigen::Vector2f d1 = a2 - a1, d2 = b2 - b1, d3 = b1 - a1;
    const float denom = cross2(d1, d2);
    if (std::abs(denom) < 1e-10f) return false;
    const float t = cross2(d3, d2) / denom;
    const float u = cross2(d3, d1) / denom;
    constexpr float eps = 1e-5f;
    return (t > eps && t < 1.f - eps && u > eps && u < 1.f - eps);
}

// =====================================================================
// Boundary crossing checks
// =====================================================================
bool PolygonPathPlanner::segment_crosses_original_boundary(
    const Eigen::Vector2f& a, const Eigen::Vector2f& b) const
{
    const int n = static_cast<int>(polygon_.size());
    for (int i = 0; i < n; ++i)
        if (segments_intersect_proper(a, b, polygon_[i], polygon_[(i + 1) % n]))
            return true;
    return false;
}

bool PolygonPathPlanner::segment_crosses_boundary(
    const Eigen::Vector2f& a, const Eigen::Vector2f& b) const
{
    // Check against the INNER polygon boundary
    const int n = static_cast<int>(inner_polygon_.size());
    for (int i = 0; i < n; ++i)
        if (segments_intersect_proper(a, b, inner_polygon_[i], inner_polygon_[(i + 1) % n]))
            return true;
    return false;
}

// =====================================================================
// Clearance check
// =====================================================================
bool PolygonPathPlanner::has_clearance(const Eigen::Vector2f& p) const
{
    if (!point_in_polygon(p, polygon_)) return false;
    return dist_to_polygon_boundary(p, polygon_) >= params.robot_radius * 0.95f;
}

// =====================================================================
// Visibility: segment must not cross the original walls NOR the inner polygon
// =====================================================================
bool PolygonPathPlanner::is_visible(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const
{
    if (polygon_.empty() || inner_polygon_.empty()) return false;
    if (!point_in_polygon(a, polygon_) || !point_in_polygon(b, polygon_))
        return false;
    // Must not cross any original wall
    if (segment_crosses_original_boundary(a, b))
        return false;
    // Must not cross the inner boundary polygon
    if (segment_crosses_boundary(a, b))
        return false;
    // Sample points: all must be inside the original polygon
    const float seg_len = (b - a).norm();
    const int num_samples = std::max(10, static_cast<int>(seg_len / 0.15f));
    for (int k = 1; k < num_samples; ++k)
    {
        const float t = static_cast<float>(k) / static_cast<float>(num_samples);
        const Eigen::Vector2f sample = a + t * (b - a);
        if (!point_in_polygon(sample, polygon_))
            return false;
    }
    return true;
}

bool PolygonPathPlanner::is_inside(const Eigen::Vector2f& point) const
{
    return !polygon_.empty() && has_clearance(point);
}

// =====================================================================
// Subdivide polygon: insert extra points on edges longer than max_edge_len
// so no edge exceeds that length. Original vertices are preserved.
// =====================================================================
std::vector<Eigen::Vector2f> PolygonPathPlanner::subdivide_polygon(
    const std::vector<Eigen::Vector2f>& poly, float max_edge_len)
{
    const int n = static_cast<int>(poly.size());
    if (n < 3) return poly;

    std::vector<Eigen::Vector2f> result;
    result.reserve(n * 2);

    for (int i = 0; i < n; ++i)
    {
        const Eigen::Vector2f& a = poly[i];
        const Eigen::Vector2f& b = poly[(i + 1) % n];
        result.push_back(a);

        float len = (b - a).norm();
        if (len > max_edge_len)
        {
            int num_segments = static_cast<int>(std::ceil(len / max_edge_len));
            for (int s = 1; s < num_segments; ++s)
            {
                float t = static_cast<float>(s) / static_cast<float>(num_segments);
                result.push_back(a + t * (b - a));
            }
        }
    }
    return result;
}

// =====================================================================
// Generate inner polygon: one node per vertex, displaced inward by offset.
// For each vertex: sweep 360° at a fixed radius of exactly offset,
// pick the direction where the candidate is inside and has best clearance.
// Falls back to shorter radii if offset doesn't fit.
// Returns exactly N nodes (1:1 with input polygon vertices).
// =====================================================================
std::vector<Eigen::Vector2f> PolygonPathPlanner::offset_polygon_inward(
    const std::vector<Eigen::Vector2f>& poly, float offset)
{
    const int n = static_cast<int>(poly.size());
    if (n < 3) return {};

    std::vector<Eigen::Vector2f> result(n);

    for (int i = 0; i < n; ++i)
    {
        Eigen::Vector2f best = poly[i];
        float best_clearance = 0.f;
        constexpr int num_angles = 36;   // every 10°

        // First pass: try at exactly offset distance
        for (int a = 0; a < num_angles; ++a)
        {
            float angle = 2.f * static_cast<float>(M_PI) * static_cast<float>(a) / static_cast<float>(num_angles);
            Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
            Eigen::Vector2f cand = poly[i] + dir * offset;
            if (point_in_polygon(cand, poly))
            {
                float c = dist_to_polygon_boundary(cand, poly);
                if (c > best_clearance)
                { best = cand; best_clearance = c; }
            }
        }

        // If no valid point at offset distance, try shorter radii
        if (best_clearance < 0.01f)
        {
            for (int a = 0; a < num_angles; ++a)
            {
                float angle = 2.f * static_cast<float>(M_PI) * static_cast<float>(a) / static_cast<float>(num_angles);
                Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
                for (int r = 9; r >= 1; --r)  // from 0.9*offset down to 0.1*offset
                {
                    float radius = offset * static_cast<float>(r) / 10.f;
                    Eigen::Vector2f cand = poly[i] + dir * radius;
                    if (point_in_polygon(cand, poly))
                    {
                        float c = dist_to_polygon_boundary(cand, poly);
                        if (c > best_clearance)
                        { best = cand; best_clearance = c; }
                    }
                }
            }
        }

        result[i] = best;
    }
    return result;
}

// =====================================================================
// Set polygon and rebuild
// =====================================================================
void PolygonPathPlanner::set_polygon(const std::vector<Eigen::Vector2f>& vertices)
{
    polygon_ = vertices;

    // 0. Subdivide polygon so no edge is longer than ~1m
    subdivided_polygon_ = subdivide_polygon(polygon_, 1.0f);
    const int ns = static_cast<int>(subdivided_polygon_.size());

    std::cout << "[PathPlanner] Original: " << polygon_.size()
              << " vertices, subdivided: " << ns << " vertices" << std::endl;

    // 1. Build inner polygon from SUBDIVIDED polygon (1:1 mapping)
    inner_polygon_ = offset_polygon_inward(subdivided_polygon_, params.robot_radius);

    // 2. Build nav nodes: inner polygon vertices + edge midpoint nodes
    shrunk_polygon_ = inner_polygon_;

    // Determine winding for edge normals (use original polygon)
    float signed_area = 0.f;
    for (int i = 0; i < static_cast<int>(polygon_.size()); ++i)
    {
        const auto& a = polygon_[i];
        const auto& b = polygon_[(i + 1) % static_cast<int>(polygon_.size())];
        signed_area += (b.x() - a.x()) * (b.y() + a.y());
    }
    const float winding = (signed_area < 0) ? -1.f : 1.f;

    // Add edge midpoint nodes (from subdivided polygon for better coverage)
    for (int i = 0; i < ns; ++i)
    {
        const Eigen::Vector2f& a = subdivided_polygon_[i];
        const Eigen::Vector2f& b = subdivided_polygon_[(i + 1) % ns];
        Eigen::Vector2f edge = b - a;
        float len = edge.norm();
        if (len < 1e-6f) continue;
        edge /= len;

        Eigen::Vector2f normal = (winding > 0)
            ? Eigen::Vector2f(-edge.y(), edge.x())
            : Eigen::Vector2f(edge.y(), -edge.x());

        Eigen::Vector2f mid = 0.5f * (a + b);
        Eigen::Vector2f candidate = mid + normal * params.robot_radius;

        if (point_in_polygon(candidate, polygon_) &&
            dist_to_polygon_boundary(candidate, polygon_) >= params.robot_radius * 0.5f)
        {
            shrunk_polygon_.push_back(candidate);
        }
        else
        {
            Eigen::Vector2f best = mid;
            float best_clearance = 0.f;
            constexpr int num_steps = 20;
            for (int s = 1; s <= num_steps; ++s)
            {
                float d = params.robot_radius * static_cast<float>(s) / static_cast<float>(num_steps);
                Eigen::Vector2f cand = mid + normal * d;
                if (point_in_polygon(cand, polygon_))
                {
                    float c = dist_to_polygon_boundary(cand, polygon_);
                    if (c > best_clearance)
                    { best = cand; best_clearance = c; }
                }
            }
            if (best_clearance > 0.01f)
                shrunk_polygon_.push_back(best);
        }
    }

    int good = 0;
    for (const auto& node : shrunk_polygon_)
    {
        float c = dist_to_polygon_boundary(node, polygon_);
        if (c >= params.robot_radius * 0.7f) good++;
    }
    std::cout << "[PathPlanner] Inner polygon: " << inner_polygon_.size()
              << ", total nav nodes: " << shrunk_polygon_.size()
              << " (" << good << " with full clearance)" << std::endl;

    if (shrunk_polygon_.size() >= 3)
        build_visibility_graph();
    else
        adjacency_.clear();
}

// =====================================================================
// Build Visibility Graph
// =====================================================================
void PolygonPathPlanner::build_visibility_graph()
{
    const int n = static_cast<int>(shrunk_polygon_.size());
    adjacency_.assign(n, {});

    int edges_added = 0;
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            if (is_visible(shrunk_polygon_[i], shrunk_polygon_[j]))
            {
                float cost = (shrunk_polygon_[i] - shrunk_polygon_[j]).norm();
                adjacency_[i].push_back({j, cost});
                adjacency_[j].push_back({i, cost});
                edges_added++;
            }
        }
    }
    std::cout << "[PathPlanner] Visibility graph: " << n << " nodes, "
              << edges_added << " edges" << std::endl;
}

// =====================================================================
// Dijkstra
// =====================================================================
std::vector<int> PolygonPathPlanner::dijkstra(
    const std::vector<std::vector<Edge>>& adj, int start, int goal)
{
    const int n = static_cast<int>(adj.size());
    std::vector<float> dist(n, std::numeric_limits<float>::infinity());
    std::vector<int> parent(n, -1);
    dist[start] = 0.f;
    using PQEntry = std::pair<float, int>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> pq;
    pq.push({0.f, start});
    while (!pq.empty())
    {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == goal) break;
        for (const auto& [to, cost] : adj[u])
        {
            float nd = d + cost;
            if (nd < dist[to])
            {
                dist[to] = nd;
                parent[to] = u;
                pq.push({nd, to});
            }
        }
    }
    return parent;
}

// =====================================================================
// Plan path
// =====================================================================
std::vector<Eigen::Vector2f> PolygonPathPlanner::plan(
    const Eigen::Vector2f& start, const Eigen::Vector2f& goal) const
{
    if (polygon_.size() < 3) return {};
    if (!point_in_polygon(start, polygon_) || !point_in_polygon(goal, polygon_))
        return {};
    if (is_visible(start, goal))
        return {start, goal};

    const int n = static_cast<int>(shrunk_polygon_.size());
    const int si = n, gi = n + 1;
    auto adj = adjacency_;
    adj.resize(n + 2);

    for (int i = 0; i < n; ++i)
    {
        if (is_visible(start, shrunk_polygon_[i]))
        {
            float c = (start - shrunk_polygon_[i]).norm();
            adj[si].push_back({i, c});
            adj[i].push_back({si, c});
        }
        if (is_visible(goal, shrunk_polygon_[i]))
        {
            float c = (goal - shrunk_polygon_[i]).norm();
            adj[gi].push_back({i, c});
            adj[i].push_back({gi, c});
        }
    }

    auto parent = dijkstra(adj, si, gi);

    std::cout << "[PathPlanner] start connects to " << adj[si].size()
              << " nodes, goal connects to " << adj[gi].size()
              << " nodes" << std::endl;

    if (parent[gi] == -1)
    {
        std::cout << "[PathPlanner] NO PATH: graph disconnected" << std::endl;
        return {};
    }

    std::vector<int> idx_path;
    for (int v = gi; v != -1; v = parent[v])
        idx_path.push_back(v);
    std::reverse(idx_path.begin(), idx_path.end());

    auto idx_to_pt = [&](int idx) -> Eigen::Vector2f {
        if (idx == si) return start;
        if (idx == gi) return goal;
        return shrunk_polygon_[idx];
    };

    std::vector<Eigen::Vector2f> path;
    float total = 0.f;
    for (size_t i = 0; i < idx_path.size(); ++i)
    {
        path.push_back(idx_to_pt(idx_path[i]));
        if (i > 0) total += (path[i] - path[i - 1]).norm();
    }
    if (total > params.max_path_length) return {};
    return path;
}

} // namespace rc

