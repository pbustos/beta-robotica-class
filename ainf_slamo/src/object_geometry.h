#pragma once
//
// object_geometry.h — Shared geometry constants for all furniture types.
//
// Consumed by:
//   • SDF analytic models   (object_models/*.cpp)
//   • 2D footprint generators (object_footprints.h)
//   • EM height lookup        (em_manager.cpp)
//   • Object palette defaults (object_palette_panel.cpp)
//

#include <string>
#include <QString>

namespace rc::geometry
{

// =====================================================================
// Per-type SDF / footprint shape ratios
// =====================================================================

// ---- Table ----
namespace table
{
    inline constexpr float top_thickness_ratio = 0.08f;   // fraction of height
    inline constexpr float top_thickness_min   = 0.03f;
    inline constexpr float leg_thickness_ratio = 0.12f;   // fraction of min(w,d)
    inline constexpr float leg_thickness_min   = 0.03f;
    inline constexpr float leg_height_min      = 0.05f;
}

// ---- Chair ----
namespace chair
{
    inline constexpr float seat_thickness_ratio    = 0.09f;   // fraction of seat_height
    inline constexpr float seat_thickness_min      = 0.03f;
    inline constexpr float backrest_height_ratio   = 0.45f;   // fraction of seat_height
    inline constexpr float backrest_height_min     = 0.20f;
    inline constexpr float backrest_thickness_ratio = 0.06f;  // fraction of depth
    inline constexpr float backrest_thickness_min  = 0.04f;
    inline constexpr float leg_size_ratio          = 0.10f;   // fraction of min(w,d)
    inline constexpr float leg_size_min            = 0.03f;
    inline constexpr float leg_height_min          = 0.05f;
}

// ---- Bench ----
namespace bench
{
    inline constexpr float seat_thickness_ratio    = 0.08f;   // fraction of height
    inline constexpr float seat_thickness_min      = 0.04f;
    inline constexpr float seat_height_ratio       = 0.45f;   // fraction of height
    inline constexpr float backrest_thickness_ratio = 0.08f;  // fraction of depth
    inline constexpr float backrest_thickness_min  = 0.04f;
    inline constexpr float backrest_height_min     = 0.08f;
    inline constexpr float support_thickness_ratio = 0.12f;   // fraction of width
    inline constexpr float support_thickness_min   = 0.04f;
    inline constexpr float support_spread          = 0.42f;   // fraction of width for ±X offset
    inline constexpr float support_depth_ratio     = 0.425f;  // 0.5 * 0.85 — fraction of depth
}

// ---- Pot (truncated cone) ----
namespace pot
{
    inline constexpr float taper_ratio      = 0.65f;  // bottom_radius / top_radius
    inline constexpr int   footprint_sides  = 16;     // n-gon approximation for 2D
}

// =====================================================================
// Default nominal sizes  (width, depth, height)
// =====================================================================

struct NominalSize { float width, depth, height; };

inline constexpr NominalSize nominal_chair   { 0.50f, 0.50f, 0.95f };
inline constexpr NominalSize nominal_table   { 1.20f, 0.80f, 0.75f };
inline constexpr NominalSize nominal_bench   { 1.50f, 0.60f, 0.48f };
inline constexpr NominalSize nominal_pot     { 0.40f, 0.40f, 0.60f };

// =====================================================================
// Expected height from label  (shared by EM manager & palette)
// =====================================================================

inline float height_from_label(const std::string& label)
{
    const QString ql = QString::fromStdString(label).toLower();
    if (ql.contains("silla")   || ql.contains("chair"))   return nominal_chair.height;
    if (ql.contains("mesa")    || ql.contains("table"))    return nominal_table.height;
    if (ql.contains("banco")   || ql.contains("bench"))    return nominal_bench.height;
    if (ql.contains("monitor") || ql.contains("pantalla") || ql.contains("screen")) return 1.10f;
    if (ql.contains("vitrina") || ql.contains("cabinet")  || ql.contains("shelf"))  return 1.60f;
    if (ql.contains("maceta")  || ql.contains("plant")    || ql.contains("pot"))    return nominal_pot.height;
    return 0.85f;
}

} // namespace rc::geometry
