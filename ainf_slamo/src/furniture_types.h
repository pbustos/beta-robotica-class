#ifndef FURNITURE_TYPES_H
#define FURNITURE_TYPES_H

#include <Eigen/Eigen>

#include <optional>
#include <string>
#include <vector>

namespace rc
{
struct FurniturePolygonData
{
    std::string id;
    std::string label;
    std::vector<Eigen::Vector2f> vertices;
    std::optional<float> last_fit_sdf;
    float frame_yaw_inward_rad = 0.f;
    float height = 0.8f;
};

} // namespace rc

#endif // FURNITURE_TYPES_H
