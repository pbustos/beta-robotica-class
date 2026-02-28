#include "polygon_path_planner.h"
#include <algorithm>
#include <queue>
#include <cmath>
#include <limits>
namespace rc
{
std::vector<Eigen::Vector2f> PolygonPathPlanner::offset_polygon_inward(
    const std::vector<Eigen::Vector2f>& poly, float offset)
{
    const int n = static_cast<int>(poly.size());
    if (n < 3) return {};
    // Determine winding via signed area
    float signed_area = 0.f;
    for (int i = 0; i < n; ++i)
    {
        const auto& a = poly[i];
        const auto& b = poly[(i + 1) % n];
        signed_area += (b.x() - a.x()) * (b.y() + a.y());
    }
    const float winding = (signed_area < 0) ? -1.f : 1.f; // -1=CCW, +1=CW
    struct OffsetLine { Eigen::Vector2f point; Eigen::Vector2f dir; };
    std::vector<OffsetLine> lines(n);
    for (int i = 0; i < n; ++i)
    {
        const Eigen::Vector2f& a = poly[i];
        const Eigen::Vector2f& b = poly[(i + 1) % n];
        Eigen::Vector2f edge = b - a;
        float len = edge.norm();
        if (len < 1e-8f) len = 1e-8f;
        edge /= len;
        Eigen::Vector2f normal = (winding > 0)
            ? Eigen::Vector2f(-edge.y(), edge.x())   // CW inward
            : Eigen::Vector2f(edge.y(), -edge.x());  // CCW inward
        lines[i].point = a + normal * offset;
        lines[i].dir = edge;
    }
    // Intersect consecutive offset lines
    std::vector<Eigen::Vector2f> result;
    result.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        const auto& l1 = lines[i];
        const auto& l2 = lines[(i + 1) % n];
        float det = l1.dir.x() * (-l2.dir.y()) - l1.dir.y() * (-l2.dir.x());
        if (std::abs(det) < 1e-10f)
        {
            result.push_back(0.5f * (l1.point + l2.point));
            continue;
        }
        Eigen::Vector2f rhs = l2.point - l1.point;
        float t = (rhs.x() * (-l2.dir.y()) - rhs.y() * (-l2.dir.x())) / det;
        result.push_back(l1.point + t * l1.dir);
    }
    // Clamp vertices that ended up outside the original polygon
    for (auto& v : result)
    {
        if (!point_in_polygon(v, poly))
        {
            float best_dist = std::numeric_limits<float>::max();
            Eigen::Vector2f best_pt = v;
            for (int i = 0; i < n; ++i)
            {
                const Eigen::Vector2f& a = poly[i];
                const Eigen::Vector2f& b = poly[(i + 1) % n];
                Eigen::Vector2f ab = b - a;
                float t_proj = (v - a).dot(ab) / (ab.squaredNorm() + 1e-10f);
                t_proj = std::clamp(t_proj, 0.f, 1.f);
                Eigen::Vector2f closest = a + t_proj * ab;
                float d = (v - closest).norm();
                if (d < best_dist) { best_dist = d; best_pt = closest; }
            }
            v = best_pt;
        }
    }
    return result;
}
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
bool PolygonPathPlanner::segment_crosses_boundary(
    const Eigen::Vector2f& a, const Eigen::Vector2f& b) const
{
    const int n = static_cast<int>(shrunk_polygon_.size());
    for (int i = 0; i < n; ++i)
        if (segments_intersect_proper(a, b, shrunk_polygon_[i], shrunk_polygon_[(i + 1) % n]))
            return true;
    return false;
}
bool PolygonPathPlanner::is_visible(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const
{
    if (shrunk_polygon_.empty()) return false;
    if (!point_in_polygon(a, shrunk_polygon_) || !point_in_polygon(b, shrunk_polygon_))
        return false;
    if (segment_crosses_boundary(a, b))
        return false;
    const Eigen::Vector2f mid = 0.5f * (a + b);
    return point_in_polygon(mid, shrunk_polygon_);
}
bool PolygonPathPlanner::is_inside(const Eigen::Vector2f& point) const
{
    return !shrunk_polygon_.empty() && point_in_polygon(point, shrunk_polygon_);
}
void PolygonPathPlanner::set_polygon(const std::vector<Eigen::Vector2f>& vertices)
{
    polygon_ = vertices;
    shrunk_polygon_ = offset_polygon_inward(polygon_, params.robot_radius);
    if (shrunk_polygon_.size() >= 3)
        build_visibility_graph();
    else
        adjacency_.clear();
}
void PolygonPathPlanner::build_visibility_graph()
{
    const int n = static_cast<int>(shrunk_polygon_.size());
    adjacency_.assign(n, {});
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
        {
            bool adjacent = (j == i + 1) || (i == 0 && j == n - 1);
            if (adjacent || is_visible(shrunk_polygon_[i], shrunk_polygon_[j]))
            {
                float cost = (shrunk_polygon_[i] - shrunk_polygon_[j]).norm();
                adjacency_[i].push_back({j, cost});
                adjacency_[j].push_back({i, cost});
            }
        }
}
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
std::vector<Eigen::Vector2f> PolygonPathPlanner::plan(
    const Eigen::Vector2f& start, const Eigen::Vector2f& goal) const
{
    if (shrunk_polygon_.size() < 3) return {};
    if (!point_in_polygon(start, shrunk_polygon_) ||
        !point_in_polygon(goal, shrunk_polygon_))
        return {};
    // Direct line of sight
    if (is_visible(start, goal))
        return {start, goal};
    // Extended graph: polygon vertices + start(n) + goal(n+1)
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
    if (parent[gi] == -1) return {};
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
