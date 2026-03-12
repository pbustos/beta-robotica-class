#ifndef SCENE_GRAPH_ADAPTER_H
#define SCENE_GRAPH_ADAPTER_H

#include "furniture_types.h"
#include "scene_graph_model.h"
#include <functional>
#include <vector>

namespace rc
{
class SceneGraphAdapter
{
public:
    static std::vector<SceneGraphObject> to_scene_objects(
        const std::vector<FurniturePolygonData>& furniture,
        const std::function<float(const std::string&)>& height_from_label);

    static void apply_scene_objects(
        const std::vector<SceneGraphObject>& scene_objects,
        std::vector<FurniturePolygonData>& furniture);

    static std::vector<FurniturePolygonData> to_furniture(
        const std::vector<SceneGraphObject>& scene_objects);

    static void rebuild_graph(
        SceneGraphModel& graph,
        const std::vector<Eigen::Vector2f>& room_polygon,
        const std::vector<FurniturePolygonData>& furniture,
        const std::function<float(const std::string&)>& height_from_label,
        float room_height = 2.6f);

    static bool accept_fit_if_improved(
        SceneGraphModel& graph,
        FurniturePolygonData& furniture_item,
        const std::vector<Eigen::Vector2f>& fitted_vertices,
        float fitted_sdf);
};

} // namespace rc

#endif // SCENE_GRAPH_ADAPTER_H
