#pragma once
//
// object_footprints.h — 2D footprint polygon generators for each furniture type.
//
// All functions use the *standard* 2D rotation convention:
//   world = center + R(yaw) * local
// where R(yaw) = [[cos, -sin], [sin, cos]].
//
// Width is the extent along the local X axis, depth along local Y.
// The "facing" direction (front of the object) points along +Y_local,
// which in world space is (-sin yaw, cos yaw).
//
// These footprints are the authoritative 2D representation — used by
// both Viewer2D and Viewer3D to guarantee visual consistency.
//

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "object_geometry.h"

namespace rc::footprints
{

// ---- helper: rotate a set of local 2D points to world ----
inline std::vector<Eigen::Vector2f>
to_world(const std::vector<Eigen::Vector2f>& local,
         float tx, float ty, float yaw)
{
    const float c = std::cos(yaw), s = std::sin(yaw);
    std::vector<Eigen::Vector2f> out;
    out.reserve(local.size());
    for (const auto& p : local)
        out.emplace_back(tx + c * p.x() - s * p.y(),
                         ty + s * p.x() + c * p.y());
    return out;
}

// ---- Table / generic rectangular footprint ----
inline std::vector<Eigen::Vector2f>
table(float tx, float ty, float yaw, float w, float d)
{
    const float hw = w * 0.5f, hd = d * 0.5f;
    return to_world({{-hw, -hd}, {hw, -hd}, {hw, hd}, {-hw, hd}},
                    tx, ty, yaw);
}

// ---- Chair: rectangle + small backrest tick on the rear edge (+Y local) ----
inline std::vector<Eigen::Vector2f>
chair(float tx, float ty, float yaw, float w, float d)
{
    const float hw = w * 0.5f, hd = d * 0.5f;
    const float bt = geometry::chair::backrest_thickness_ratio * d;  // backrest thickness inset
    //  5----4
    //  | bb |   <- backrest notch (thin strip at -Y side)
    //  6    3
    //  |    |
    //  7----2
    //     ^front (+Y)
    // Actually: backrest at -Y (rear). Notch shows a thin line there.
    return to_world({
        {-hw,  hd},          // 0: front-left
        { hw,  hd},          // 1: front-right
        { hw, -hd},          // 2: rear-right
        { hw - bt, -hd},     // 3: notch in
        { hw - bt, -hd + bt},// 4: notch up
        {-hw + bt, -hd + bt},// 5: notch across
        {-hw + bt, -hd},     // 6: notch out
        {-hw, -hd},          // 7: rear-left
    }, tx, ty, yaw);
}

// ---- Bench: rectangle with backrest strip along rear (-Y) edge (park bench) ----
inline std::vector<Eigen::Vector2f>
bench(float tx, float ty, float yaw, float w, float d)
{
    const float hw = w * 0.5f, hd = d * 0.5f;
    const float bt = geometry::bench::backrest_thickness_ratio * d;  // backrest thickness
    //  6-----------5
    //  | backrest  |   <- thin strip at -Y (rear)
    //  7           4
    //  |   seat    |
    //  0-----------3
    //     ^front (+Y)
    return to_world({
        {-hw,  hd},           // 0: front-left
        { hw,  hd},           // 1: front-right
        { hw, -hd},           // 2: rear-right
        { hw - bt, -hd},     // 3: notch in
        { hw - bt, -hd + bt},// 4: notch up
        {-hw + bt, -hd + bt},// 5: notch across
        {-hw + bt, -hd},     // 6: notch out
        {-hw, -hd},          // 7: rear-left
    }, tx, ty, yaw);
}

// ---- Pot (maceta): circular footprint from top radius of truncated cone ----
// The projected polygon is a circle with radius = max(w, d) / 2  (the wider
// top opening of the truncated cone).  Approximated as a 16-gon.
inline std::vector<Eigen::Vector2f>
pot(float tx, float ty, float yaw, float w, float d)
{
    const float r = std::max(w, d) * 0.5f;
    constexpr int N = geometry::pot::footprint_sides;
    std::vector<Eigen::Vector2f> local;
    local.reserve(N);
    for (int i = 0; i < N; ++i)
    {
        const float a = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(N);
        local.emplace_back(r * std::cos(a), r * std::sin(a));
    }
    return to_world(local, tx, ty, yaw);
}

// ---- Monitor: thin rectangle ----
inline std::vector<Eigen::Vector2f>
monitor(float tx, float ty, float yaw, float w, float d)
{
    return table(tx, ty, yaw, w, d);
}

// ---- Cabinet: rectangle ----
inline std::vector<Eigen::Vector2f>
cabinet(float tx, float ty, float yaw, float w, float d)
{
    return table(tx, ty, yaw, w, d);
}

// ---- Dispatch by object_type string ----
inline std::vector<Eigen::Vector2f>
make(const std::string& object_type,
     float tx, float ty, float yaw, float w, float d)
{
    if (object_type == "table")   return table(tx, ty, yaw, w, d);
    if (object_type == "chair")   return chair(tx, ty, yaw, w, d);
    if (object_type == "bench")   return bench(tx, ty, yaw, w, d);
    if (object_type == "pot")     return pot(tx, ty, yaw, w, d);
    // fallback: rectangle
    return table(tx, ty, yaw, w, d);
}

} // namespace rc::footprints
