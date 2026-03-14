#pragma once

#include "furniture_types.h"
#include "scene_graph_model.h"
#include "object_footprints.h"

#include <Eigen/Dense>
#include <QObject>
#include <functional>
#include <optional>
#include <string>
#include <vector>

class LayoutManager : public QObject
{
    Q_OBJECT
public:
    explicit LayoutManager(rc::SceneGraphModel& scene_graph, QObject* parent = nullptr);

    // ---- Room polygon ----
    const std::vector<Eigen::Vector2f>& room_polygon() const { return room_polygon_; }
    void set_room_polygon(std::vector<Eigen::Vector2f> poly);

    /// Backup current polygon (for capture mode)
    void backup_room_polygon();
    /// Restore backed-up polygon (cancel capture)
    void restore_room_polygon();
    /// Begin new room capture (clears current polygon)
    void begin_capture();
    /// Append a vertex during room capture
    void capture_vertex(const Eigen::Vector2f& v);

    // ---- Furniture ----
    const std::vector<rc::FurniturePolygonData>& furniture() const { return furniture_; }
    std::vector<rc::FurniturePolygonData>& furniture_mutable() { return furniture_; }
    std::size_t furniture_count() const { return furniture_.size(); }

    /// Add a new furniture item.  Returns its index.
    std::size_t add_furniture(rc::FurniturePolygonData fp);
    /// Remove furniture by label.  Returns true if found.
    bool remove_furniture(const std::string& label);

    /// Recompute a single item's footprint vertices from scene_graph node.
    void refresh_item_from_model(std::size_t idx);
    /// Recompute ALL footprint vertices from scene_graph nodes.
    void refresh_all_from_model();
    /// Replace furniture list wholesale (e.g. after USD load).
    void replace_all(std::vector<rc::FurniturePolygonData> items);

    // ---- Obstacle extraction (for path planner) ----
    std::vector<std::vector<Eigen::Vector2f>> obstacle_polygons() const;

    // ---- Graph access ----
    rc::SceneGraphModel& scene_graph() { return scene_graph_; }
    const rc::SceneGraphModel& scene_graph() const { return scene_graph_; }

    // ---- Persistence helpers ----
    static constexpr const char* PERSISTED_SCENE_GRAPH_FILE = "./scene_graph.usda";

Q_SIGNALS:
    /// Emitted after any furniture mutation (add/remove/refresh/replace).
    void furnitureChanged();
    /// Emitted after a single furniture item changed (index still valid).
    void furnitureItemChanged(std::size_t index);
    /// Emitted after room polygon changed.
    void roomChanged();

private:
    rc::SceneGraphModel& scene_graph_;
    std::vector<rc::FurniturePolygonData> furniture_;
    std::vector<Eigen::Vector2f> room_polygon_;
    std::vector<Eigen::Vector2f> room_polygon_backup_;
};
