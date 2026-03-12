#include "specificworker.h"

#include <cmath>
#include <cstdint>
#include <cstring>

bool SpecificWorker::extract_depth_buffer_meters(const RoboCompImageSegmentation::TDepth& depth,
                                                 int& width,
                                                 int& height,
                                                 std::vector<float>& out)
{
    width = depth.width;
    height = depth.height;
    if (width <= 0 || height <= 0)
        return false;

    const std::size_t expected = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (expected == 0)
        return false;

    const std::size_t nbytes = depth.depth.size();
    const float scale = (std::abs(depth.depthFactor) > 1e-9f) ? depth.depthFactor : 1.f;

    out.assign(expected, 0.f);

    // Preferred format: packed float32 depth image in meters (or scaled by depthFactor).
    if (nbytes >= expected * sizeof(float))
    {
        for (std::size_t i = 0; i < expected; ++i)
        {
            float v = 0.f;
            std::memcpy(&v, depth.depth.data() + i * sizeof(float), sizeof(float));
            out[i] = std::isfinite(v) ? v * scale : 0.f;
        }
    }
    // Fallback: uint16 depth (usually millimeters).
    else if (nbytes >= expected * sizeof(std::uint16_t))
    {
        for (std::size_t i = 0; i < expected; ++i)
        {
            std::uint16_t mm = 0;
            std::memcpy(&mm, depth.depth.data() + i * sizeof(std::uint16_t), sizeof(std::uint16_t));
            const float v = static_cast<float>(mm);
            out[i] = std::isfinite(v) ? v : 0.f;
        }
    }
    // Fallback: one byte per pixel (legacy/debug only).
    else if (nbytes >= expected)
    {
        for (std::size_t i = 0; i < expected; ++i)
        {
            const float v = static_cast<float>(depth.depth[i]);
            out[i] = std::isfinite(v) ? v : 0.f;
        }
    }
    else
        return false;

    if (nbytes < expected * sizeof(float))
    {
        for (float& d : out)
            d = std::isfinite(d) ? d * 0.001f * scale : 0.f;
    }

    return true;
}

bool SpecificWorker::extract_depth_from_tdata(const RoboCompImageSegmentation::TData& tdata,
                                              int& width,
                                              int& height,
                                              std::vector<float>& out)
{
    return extract_depth_buffer_meters(tdata.depth, width, height, out);
}
