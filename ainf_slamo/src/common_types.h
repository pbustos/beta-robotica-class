#pragma once

#include <chrono>
#include <vector>

// Minimal shared types required by room_model/room_concept code.
// Keep this header lightweight and dependency-free.

enum class RoomState
{
    MAPPING = 0,
    LOCALIZED = 1
};

struct VelocityCommand
{
    // Convention used by the existing code: adv_x and adv_z are linear velocities
    // in robot local frame, and rot is angular velocity.
    float adv_x = 0.f;
    float adv_z = 0.f;
    float rot = 0.f;
    std::chrono::time_point<std::chrono::high_resolution_clock> timestamp{};
};

using VelocityHistory = std::vector<VelocityCommand>;

