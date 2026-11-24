#ifndef CORNER_DETECTOR_H
#define CORNER_DETECTOR_H

#include "localization_types.h"
#include <Eigen/Dense>
#include <vector>

// Robot frame everywhere: x right, y forward, mm. Pure Eigen/STL.

// Finds room corners in a lidar cloud: height band + downsampling + iterative RANSAC lines (PCA refit) split into
// segments + pairing of roughly perpendicular segments that nearly meet. Corner.bisector follows RoomLayout: arms point
// from the corner along each segment toward its far end. convex = the robot (origin) lies inside the corner wedge.
class CornerDetector
{
public:
    // min_z: mm, only points at least this high are used (above furniture, the walls and corners remain).
    explicit CornerDetector(float min_z = 1800.f);

    std::vector<Corner> detect(const std::vector<Eigen::Vector3f> &points_robot);   // robot frame, mm
    const std::vector<LineSegment> &lines() const;                                  // from the last detect(), for drawing

    static constexpr float ROBOT_RADIUS = 400.f;        // mm, closer points are the robot itself and are dropped
    static constexpr float GRID = 50.f;                 // mm, one point per grid cell
    static constexpr int RANSAC_ITERS = 200;            // random line hypotheses per extracted line
    static constexpr float INLIER_DIST = 40.f;          // mm, max point-to-line distance of an inlier
    static constexpr int MIN_INLIERS = 15;              // minimum inliers for a line to be accepted (the 802 mm wall between corners 1 and 2 gives only ~20 points after downsampling)
    static constexpr int MAX_LINES = 12;                // maximum lines extracted per scan
    static constexpr float SEGMENT_GAP = 500.f;         // mm, a line's inliers are split at gaps larger than this
    static constexpr float MIN_SEGMENT_LEN = 400.f;     // mm, shorter segments are discarded
    static constexpr int MIN_SEGMENT_PTS = 10;          // minimum points of a segment
    static constexpr float ANGLE_TOL = 20.f;            // deg, corners need 90 +- this between the two lines
    static constexpr float ENDPOINT_DIST = 400.f;       // mm, max distance from the intersection to the nearest endpoint of each segment
    static constexpr float MERGE_DIST = 300.f;          // mm, corners closer than this are duplicates

private:
    std::vector<Eigen::Vector2f> prepare(const std::vector<Eigen::Vector3f> &points) const;
    std::vector<LineSegment> extract_segments(std::vector<Eigen::Vector2f> pts) const;
    std::vector<Corner> pair_segments(const std::vector<LineSegment> &segs) const;

    float min_z_;
    std::vector<LineSegment> lines_;
};

#endif // CORNER_DETECTOR_H
