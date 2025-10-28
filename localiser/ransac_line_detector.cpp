//
// Created by pbustos on 27/10/25.
//

#include "ransac_line_detector.h"
#include <random>
#include <algorithm>
#include "common_types.h"

namespace rc
{
    std::vector<LineSegment> RansacLineDetector::detect_lines(const std::vector<Eigen::Vector2d>& points, const Params& params)
    {
        if (points.size() < 2) return {};

        std::vector<LineSegment> lines;
        std::vector<bool> used(points.size(), false);
        std::random_device rd;
        std::mt19937 gen(rd());

        for (int line_idx = 0; line_idx < params.max_lines; ++line_idx) {
            // Get unused points
            std::vector<int> available;
            available.reserve(points.size());
            for (size_t i = 0; i < points.size(); ++i) {
                if (!used[i]) available.push_back(i);
            }

            if (available.size() < static_cast<size_t>(params.min_points_per_line)) break;

            // RANSAC for one line
            LineSegment best_line;
            best_line.score = 0;

            std::uniform_int_distribution<> dis(0, available.size() - 1);

            for (int iter = 0; iter < params.max_iterations; ++iter) {
                // Sample 2 random points
                int idx1 = available[dis(gen)];
                int idx2 = available[dis(gen)];
                if (idx1 == idx2) continue;

                const auto& p1 = points[idx1];
                const auto& p2 = points[idx2];

                // Line direction and normal
                Eigen::Vector2d dir = (p2 - p1).normalized();
                Eigen::Vector2d normal(-dir.y(), dir.x());

                // Find inliers
                std::vector<int> inliers;
                inliers.reserve(available.size());

                for (int idx : available) {
                    double dist = std::abs((points[idx] - p1).dot(normal));
                    if (dist < params.distance_threshold) {
                        inliers.push_back(idx);
                    }
                }

                // Check if this is the best line so far
                if (inliers.size() > best_line.score) {
                    best_line.inlier_indices = inliers;
                    best_line.score = inliers.size();
                }

                // Early termination if we found enough inliers
                if (best_line.score > available.size() * 0.8) break;
            }

            // Check if we found a valid line
            if (best_line.score < params.min_points_per_line) break;

            // Fit line to all inliers (least squares)
            Eigen::Vector2d centroid(0, 0);
            for (int idx : best_line.inlier_indices) {
                centroid += points[idx];
            }
            centroid /= best_line.inlier_indices.size();

            // Compute covariance
            double xx = 0, xy = 0, yy = 0;
            for (int idx : best_line.inlier_indices) {
                Eigen::Vector2d p = points[idx] - centroid;
                xx += p.x() * p.x();
                xy += p.x() * p.y();
                yy += p.y() * p.y();
            }

            // Principal direction (eigenvector of largest eigenvalue)
            double trace = xx + yy;
            double det = xx * yy - xy * xy;
            double lambda = (trace + std::sqrt(trace * trace - 4 * det)) / 2;
            Eigen::Vector2d direction(xy, lambda - xx);
            direction.normalize();

            // Find line endpoints (project all inliers onto line)
            double min_proj = std::numeric_limits<double>::max();
            double max_proj = std::numeric_limits<double>::lowest();

            for (int idx : best_line.inlier_indices) {
                double proj = (points[idx] - centroid).dot(direction);
                min_proj = std::min(min_proj, proj);
                max_proj = std::max(max_proj, proj);
            }

            best_line.start = centroid + min_proj * direction;
            best_line.end = centroid + max_proj * direction;
            best_line.direction = direction;
            best_line.num_inliers = best_line.inlier_indices.size();  // Explicit count

            // Check minimum length
            double length = (best_line.end - best_line.start).norm();
            if (length < params.min_line_length) break;

            // Mark points as used
            for (int idx : best_line.inlier_indices)
                used[idx] = true;

            lines.push_back(best_line);
        }
        // TODO: if lines are parallel and very close, keep only the one with more inliers (non-maximum suppression)

        return lines;
    }
} // rc