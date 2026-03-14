#include "layout_manager.h"

LayoutManager::LayoutManager(rc::SceneGraphModel& scene_graph, QObject* parent)
    : QObject(parent)
    , scene_graph_(scene_graph)
{
}

// ---------------------------------------------------------------------------
// Room polygon
// ---------------------------------------------------------------------------

void LayoutManager::set_room_polygon(std::vector<Eigen::Vector2f> poly)
{
    room_polygon_ = std::move(poly);
    emit roomChanged();
}

void LayoutManager::backup_room_polygon()
{
    room_polygon_backup_ = room_polygon_;
}

void LayoutManager::restore_room_polygon()
{
    room_polygon_ = room_polygon_backup_;
    emit roomChanged();
}

void LayoutManager::begin_capture()
{
    room_polygon_.clear();
}

void LayoutManager::capture_vertex(const Eigen::Vector2f& v)
{
    room_polygon_.push_back(v);
}

// ---------------------------------------------------------------------------
// Furniture
// ---------------------------------------------------------------------------

std::size_t LayoutManager::add_furniture(rc::FurniturePolygonData fp)
{
    furniture_.push_back(std::move(fp));
    const std::size_t idx = furniture_.size() - 1;
    emit furnitureChanged();
    return idx;
}

bool LayoutManager::remove_furniture(const std::string& label)
{
    auto it = std::find_if(furniture_.begin(), furniture_.end(),
                           [&](const rc::FurniturePolygonData& f) { return f.label == label; });
    if (it == furniture_.end())
        return false;
    furniture_.erase(it);
    emit furnitureChanged();
    return true;
}

void LayoutManager::refresh_item_from_model(std::size_t idx)
{
    if (idx >= furniture_.size())
        return;

    auto& fp = furniture_[idx];
    auto node_opt = scene_graph_.get_object_node(fp.label);
    if (!node_opt)
        return;

    const auto& nd = *node_opt;
    fp.vertices = rc::footprints::make(
        nd.object_type,
        nd.translation.x(), nd.translation.y(),
        nd.yaw_rad,
        std::max(0.08f, nd.extents.x()),
        std::max(0.08f, nd.extents.y()));
    fp.frame_yaw_inward_rad = nd.yaw_rad;
    fp.height = std::max(0.2f, nd.extents.z());

    emit furnitureItemChanged(idx);
}

void LayoutManager::refresh_all_from_model()
{
    for (std::size_t i = 0; i < furniture_.size(); ++i)
    {
        auto& fp = furniture_[i];
        auto node_opt = scene_graph_.get_object_node(fp.label);
        if (!node_opt)
            continue;

        const auto& nd = *node_opt;
        fp.vertices = rc::footprints::make(
            nd.object_type,
            nd.translation.x(), nd.translation.y(),
            nd.yaw_rad,
            std::max(0.08f, nd.extents.x()),
            std::max(0.08f, nd.extents.y()));
        fp.frame_yaw_inward_rad = nd.yaw_rad;
        fp.height = std::max(0.2f, nd.extents.z());
    }
    emit furnitureChanged();
}

void LayoutManager::replace_all(std::vector<rc::FurniturePolygonData> items)
{
    furniture_ = std::move(items);
    emit furnitureChanged();
}

// ---------------------------------------------------------------------------
// Obstacle extraction
// ---------------------------------------------------------------------------

std::vector<std::vector<Eigen::Vector2f>> LayoutManager::obstacle_polygons() const
{
    std::vector<std::vector<Eigen::Vector2f>> obs;
    obs.reserve(furniture_.size());
    for (const auto& fp : furniture_)
    {
        if (fp.vertices.size() >= 3)
            obs.push_back(fp.vertices);
    }
    return obs;
}
