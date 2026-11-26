#ifndef LOCALIZATION_TYPES_H
#define LOCALIZATION_TYPES_H

#include <Eigen/Dense>

// World frame: mm, rad. Robot frame: x right, y forward, mm.

struct Pose2D
{
    float x = 0, y = 0, theta = 0;                        // world, mm, rad
};

// A room corner. bisector = normalize(a + b), a and b being the unit arm directions from the corner along its walls.
struct Corner
{
    Eigen::Vector2f p = Eigen::Vector2f::Zero();          // mm
    Eigen::Vector2f bisector = Eigen::Vector2f::Zero();   // unit
    bool convex = true;
};

struct LineSegment
{
    Eigen::Vector2f a = Eigen::Vector2f::Zero();          // mm
    Eigen::Vector2f b = Eigen::Vector2f::Zero();          // mm
    int support = 0;                                      // number of points supporting the segment
};

struct TrackResult
{
    Pose2D pose;
    int matched = 0;
    float rms = 0;                                        // mm, over matched corners
};

#endif
