#pragma once

#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

// Minimal shared types required by room_model/room_concept code.
// Keep this header lightweight and dependency-free.

namespace rc
{
    enum class RoomState
    {
        MAPPING = 0,
        LOCALIZED = 1
    };

    struct VelocityCommand
    {
        float adv_x = 0.0f;  // mm/s
        float adv_y = 0.0f;  // mm/s
        float rot = 0.0f;    // rad/s
        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
        VelocityCommand() = default;
        VelocityCommand(float x, float z, float r)
            : adv_x(x), adv_y(z), rot(r)
            , timestamp(std::chrono::high_resolution_clock::now())
        {}
    };

    using VelocityHistory = std::vector<VelocityCommand>;
    using TimeStamp = std::chrono::time_point<std::chrono::high_resolution_clock>;
}
