#ifndef EM_MANAGER_H
#define EM_MANAGER_H

#include <Eigen/Eigen>
#include <QString>
#include <QColor>
#include <functional>
#include <optional>
#include <string>
#include <vector>
#include "furniture_types.h"
#include "mesh_sdf_optimizer.h"
#include "object_ownership_em.h"
#include "scene_graph_model.h"
#include "scene_graph_adapter.h"

class CameraViewer;
class QLabel;

namespace rc { class Viewer2D; }

namespace RoboCompImageSegmentation { struct TData; struct TDepth; }

class EMManager
{
public:
    struct Context
    {
        CameraViewer* camera_viewer = nullptr;
        rc::Viewer2D* viewer_2d = nullptr;
        std::vector<rc::FurniturePolygonData>* furniture_polygons = nullptr;
        const std::vector<Eigen::Vector2f>* room_polygon = nullptr;
        rc::SceneGraphModel* scene_graph = nullptr;
        QLabel* status_label = nullptr;
        QLabel* sdf_label = nullptr;
        float camera_tx = 0.f;
        float camera_ty = 0.f;
        float camera_tz = 0.f;
        std::function<void()> on_accepted;  // draw_furniture + save
    };

    void set_context(const Context& ctx) { ctx_ = ctx; }

    // Ownership EM (lidar-based, per-frame)
    void rebuild_models();
    void run_ownership_step(const std::vector<Eigen::Vector3f>& points_robot,
                            const Eigen::Affine2f& robot_pose);
    void apply_visuals();

    // Camera EM validator (button-triggered)
    // If visible_indices is provided (from SyntheticCameraRenderer), they are used
    // instead of re-computing the frustum polygon intersection.
    void run_camera_validator(const Eigen::Affine2f& robot_pose,
                              const RoboCompImageSegmentation::TData& tdata,
                              const std::vector<int>& visible_indices = {});
    void apply_pending_adjustments(bool accept);

    // Clear overlay state (used when toggling EM off)
    void clear_overlay();

    // State queries
    bool is_enabled() const { return enabled_; }
    void set_enabled(bool e) { enabled_ = e; }
    bool is_overlay_locked() const { return overlay_lock_; }
    bool is_overlay_persistent() const { return overlay_persistent_; }
    bool is_decision_pending() const { return decision_pending_; }

    // Shared utility
    static float model_height_from_label(const std::string& label);

    // Depth extraction helpers (moved from SpecificWorker)
    static bool extract_depth_buffer_meters(const RoboCompImageSegmentation::TDepth& depth,
                                            int& width, int& height,
                                            std::vector<float>& out);
    static bool extract_depth_from_tdata(const RoboCompImageSegmentation::TData& tdata,
                                         int& width, int& height,
                                         std::vector<float>& out);

private:
    Context ctx_;
    rc::MeshSDFOptimizer mesh_sdf_optimizer_;
    rc::ObjectOwnershipEM ownership_em_;
    bool enabled_ = true;
    bool overlay_lock_ = false;
    bool overlay_persistent_ = false;
    struct PendingAdjustment
    {
        int furniture_index = -1;
        std::vector<Eigen::Vector2f> vertices;
        float new_sdf = 0.f;
        std::optional<float> prev_sdf;
        std::string label;
    };
    std::vector<PendingAdjustment> pending_adjustments_;
    bool decision_pending_ = false;
    int log_counter_ = 0;
};

#endif // EM_MANAGER_H
