#include "scene_graph_adapter.h"

#include <algorithm>
#include <cmath>

namespace rc
{
std::vector<SceneGraphObject> SceneGraphAdapter::to_scene_objects(
    const std::vector<FurniturePolygonData>& furniture,
    const std::function<float(const std::string&)>& height_from_label)
{
    std::vector<SceneGraphObject> out;
    out.reserve(furniture.size());
    for (const auto& fp : furniture)
    {
        SceneGraphObject obj;
        obj.id = fp.id;
        obj.label = fp.label;
        obj.vertices = fp.vertices;
        obj.last_fit_sdf = fp.last_fit_sdf;
        obj.frame_yaw_inward_rad = fp.frame_yaw_inward_rad;
        const float h = height_from_label ? height_from_label(fp.label) : fp.height;
        obj.height = std::max(0.2f, h);
        out.emplace_back(std::move(obj));
    }
    return out;
}

void SceneGraphAdapter::apply_scene_objects(
    const std::vector<SceneGraphObject>& scene_objects,
    std::vector<FurniturePolygonData>& furniture)
{
    const std::size_t n = std::min(scene_objects.size(), furniture.size());
    for (std::size_t i = 0; i < n; ++i)
    {
        furniture[i].vertices = scene_objects[i].vertices;
        furniture[i].last_fit_sdf = scene_objects[i].last_fit_sdf;
        furniture[i].frame_yaw_inward_rad = scene_objects[i].frame_yaw_inward_rad;
        furniture[i].height = std::max(0.2f, scene_objects[i].height);
    }
}

std::vector<FurniturePolygonData> SceneGraphAdapter::to_furniture(
    const std::vector<SceneGraphObject>& scene_objects)
{
    std::vector<FurniturePolygonData> out;
    out.reserve(scene_objects.size());
    for (const auto& so : scene_objects)
    {
        FurniturePolygonData fp;
        fp.id = so.id;
        fp.label = so.label;
        fp.vertices = so.vertices;
        fp.last_fit_sdf = so.last_fit_sdf;
        fp.frame_yaw_inward_rad = so.frame_yaw_inward_rad;
        fp.height = std::max(0.2f, so.height);
        out.emplace_back(std::move(fp));
    }
    return out;
}

void SceneGraphAdapter::rebuild_graph(
    SceneGraphModel& graph,
    const std::vector<Eigen::Vector2f>& room_polygon,
    const std::vector<FurniturePolygonData>& furniture,
    const std::function<float(const std::string&)>& height_from_label,
    float room_height)
{
    graph.rebuild(room_polygon, to_scene_objects(furniture, height_from_label), room_height);
}

bool SceneGraphAdapter::accept_fit_if_improved(
    SceneGraphModel& graph,
    FurniturePolygonData& furniture_item,
    const std::vector<Eigen::Vector2f>& fitted_vertices,
    float fitted_sdf)
{
    SceneGraphObject updated;
    const bool accepted = graph.try_update_object_fit(
        furniture_item.id,
        furniture_item.label,
        fitted_vertices,
        fitted_sdf,
        &updated);

    if (!accepted)
        return false;

    furniture_item.vertices = std::move(updated.vertices);
    furniture_item.last_fit_sdf = updated.last_fit_sdf;
    furniture_item.frame_yaw_inward_rad = updated.frame_yaw_inward_rad;
    furniture_item.height = std::max(0.2f, updated.height);
    return true;
}

} // namespace rc
