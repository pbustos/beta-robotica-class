#include "corner_detector.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_set>

CornerDetector::CornerDetector(float min_z) : min_z_(min_z) {}

const std::vector<LineSegment> &CornerDetector::lines() const
{
    return lines_;
}

std::vector<Corner> CornerDetector::detect(const std::vector<Eigen::Vector3f> &points_robot)
{
    lines_.clear();
    std::vector<Eigen::Vector2f> pts = prepare(points_robot);
    if (pts.empty()) return {};
    lines_ = extract_segments(std::move(pts));
    return pair_segments(lines_);
}

std::vector<Eigen::Vector2f> CornerDetector::prepare(const std::vector<Eigen::Vector3f> &points) const
{
    std::vector<Eigen::Vector2f> out;
    std::unordered_set<uint64_t> seen;
    for (const auto &p : points)
    {
        if (p.z() < min_z_) continue;
        const Eigen::Vector2f q(p.x(), p.y());
        if (q.norm() < ROBOT_RADIUS) continue;
        const int64_t cx = static_cast<int64_t>(std::floor(q.x() / GRID));
        const int64_t cy = static_cast<int64_t>(std::floor(q.y() / GRID));
        if (seen.insert((static_cast<uint64_t>(cx) << 32) ^ (static_cast<uint64_t>(cy) & 0xffffffffULL)).second) out.push_back(q);
    }
    return out;
}

std::vector<LineSegment> CornerDetector::extract_segments(std::vector<Eigen::Vector2f> pts) const
{
    std::vector<LineSegment> segs;
    std::mt19937 rng(12345);
    for (int n = 0; n < MAX_LINES && static_cast<int>(pts.size()) >= MIN_INLIERS; ++n)
    {
        std::uniform_int_distribution<size_t> pick(0, pts.size() - 1);
        int best_count = 0;
        Eigen::Vector2f best_p = Eigen::Vector2f::Zero(), best_n = Eigen::Vector2f::Zero();
        for (int it = 0; it < RANSAC_ITERS; ++it)
        {
            const size_t i = pick(rng), j = pick(rng);
            if (i == j) continue;
            const Eigen::Vector2f d = pts[j] - pts[i];
            if (d.norm() < 1.f) continue;
            const Eigen::Vector2f nrm(-d.y() / d.norm(), d.x() / d.norm());
            int count = 0;
            for (const auto &q : pts)
                if (std::fabs(nrm.dot(q - pts[i])) < INLIER_DIST) ++count;
            if (count > best_count) { best_count = count; best_p = pts[i]; best_n = nrm; }
        }
        if (best_count < MIN_INLIERS) break;

        // refit by PCA on the inliers of the best hypothesis
        std::vector<Eigen::Vector2f> inl;
        Eigen::Vector2f mean = Eigen::Vector2f::Zero();
        for (const auto &q : pts)
            if (std::fabs(best_n.dot(q - best_p)) < INLIER_DIST) { inl.push_back(q); mean += q; }
        mean /= static_cast<float>(inl.size());
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const auto &q : inl) cov += (q - mean) * (q - mean).transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
        const Eigen::Vector2f dir = es.eigenvectors().col(1);     // largest eigenvalue
        const Eigen::Vector2f nrm(-dir.y(), dir.x());

        // final inliers w.r.t. the refit line; they leave the pool
        std::vector<std::pair<float, Eigen::Vector2f>> on_line;
        std::vector<Eigen::Vector2f> rest;
        for (const auto &q : pts)
        {
            if (std::fabs(nrm.dot(q - mean)) < INLIER_DIST) on_line.emplace_back(dir.dot(q - mean), q);
            else rest.push_back(q);
        }
        pts = std::move(rest);
        std::sort(on_line.begin(), on_line.end(), [](const auto &a, const auto &b) { return a.first < b.first; });

        // split at gaps along the line
        size_t start = 0;
        for (size_t k = 1; k <= on_line.size(); ++k)
        {
            if (k < on_line.size() && on_line[k].first - on_line[k - 1].first <= SEGMENT_GAP) continue;
            const size_t cnt = k - start;
            const float s0 = on_line[start].first, s1 = on_line[k - 1].first;
            if (static_cast<int>(cnt) >= MIN_SEGMENT_PTS && s1 - s0 >= MIN_SEGMENT_LEN)
                segs.push_back({mean + dir * s0, mean + dir * s1, static_cast<int>(cnt)});
            start = k;
        }
    }
    return segs;
}

std::vector<Corner> CornerDetector::pair_segments(const std::vector<LineSegment> &segs) const
{
    const float max_cos = std::cos((90.f - ANGLE_TOL) * static_cast<float>(M_PI) / 180.f);
    std::vector<std::pair<int, Corner>> found;     // support, corner
    for (size_t i = 0; i < segs.size(); ++i)
        for (size_t j = i + 1; j < segs.size(); ++j)
        {
            const Eigen::Vector2f di = (segs[i].b - segs[i].a).normalized();
            const Eigen::Vector2f dj = (segs[j].b - segs[j].a).normalized();
            if (std::fabs(di.dot(dj)) > max_cos) continue;

            // intersection a_i + u*di = a_j + v*dj
            Eigen::Matrix2f M;
            M << di, -dj;
            const Eigen::Vector2f uv = M.inverse() * (segs[j].a - segs[i].a);
            const Eigen::Vector2f I = segs[i].a + uv.x() * di;

            const float ei = std::min((I - segs[i].a).norm(), (I - segs[i].b).norm());
            const float ej = std::min((I - segs[j].a).norm(), (I - segs[j].b).norm());
            if (ei > ENDPOINT_DIST || ej > ENDPOINT_DIST) continue;

            const Eigen::Vector2f far_i = (I - segs[i].a).norm() > (I - segs[i].b).norm() ? segs[i].a : segs[i].b;
            const Eigen::Vector2f far_j = (I - segs[j].a).norm() > (I - segs[j].b).norm() ? segs[j].a : segs[j].b;
            const Eigen::Vector2f a = (far_i - I).normalized(), b = (far_j - I).normalized();
            Eigen::Matrix2f A;
            A << a, b;
            const Eigen::Vector2f st = A.inverse() * (-I);

            Corner c;
            c.p = I;
            c.bisector = (a + b).normalized();
            c.convex = st.x() > 0 && st.y() > 0;
            found.push_back({segs[i].support + segs[j].support, c});
        }

    std::sort(found.begin(), found.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
    std::vector<Corner> out;
    for (const auto &f : found)
    {
        const bool dup = std::any_of(out.begin(), out.end(),
                                     [&](const Corner &c) { return (c.p - f.second.p).norm() < MERGE_DIST; });
        if (!dup) out.push_back(f.second);
    }
    return out;
}
