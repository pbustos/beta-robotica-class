
#include "viewer_3d.h"

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QDirectionalLight>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QExtrudedTextMesh>
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/QObjectPicker>
#include <Qt3DRender/QPickEvent>
#include <Qt3DRender/QPickingSettings>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DRender/QEffect>
#include <Qt3DRender/QTechnique>
#include <Qt3DRender/QRenderPass>
#include <Qt3DRender/QDepthTest>
#include <Qt3DRender/QLineWidth>
#include <QSizePolicy>
#include <QLabel>
#include <QTimer>
#include <QFont>
#include <QMouseEvent>
#include <QEvent>
#include <QCoreApplication>
#include <QDir>
#include <cmath>
#include <algorithm>

namespace rc {

// ---------------------------------------------------------------------------
// WebotsStyleCameraController::moveCamera
// Webots 3D viewport:
//   Left drag              → orbit around pivot (view centre)
//   Right drag             → pan (translate camera + view centre)
//   Middle or Left+Right   → vertical zoom + horizontal roll
//   Wheel                  → zoom (dolly)
// ---------------------------------------------------------------------------
void WebotsStyleCameraController::moveCamera(
    const Qt3DExtras::QAbstractCameraController::InputState& state, float dt)
{
    auto* cam = camera();
    if (!cam) return;

    auto dolly_clamped = [cam](float signed_step)
    {
        QVector3D to_center = cam->viewCenter() - cam->position();
        float dist = to_center.length();
        if (dist < 1e-5f)
            return;

        const QVector3D dir = to_center / dist;
        // Prevent camera from getting too close to the pivot, which can clip the gizmo.
        constexpr float min_dist = 1.10f;
        constexpr float max_dist = 250.f;

        const float new_dist = std::clamp(dist + signed_step, min_dist, max_dist);
        cam->setPosition(cam->viewCenter() - dir * new_dist);
    };

    const float look = lookSpeed() * dt;
    const float lin  = linearSpeed() * dt;
    const bool dual_button = state.leftMouseButtonActive && state.rightMouseButtonActive;
    const bool zoom_roll_mode = state.middleMouseButtonActive || dual_button;

    // ---- Orbit: left drag around current pivot ----------------------------
    if (state.leftMouseButtonActive && !state.rightMouseButtonActive && !state.middleMouseButtonActive)
    {
        const float yaw_deg = state.rxAxisValue * look;
        const float pitch_deg = -state.ryAxisValue * look;

        const QVector3D view_dir = (cam->viewCenter() - cam->position()).normalized();
        const float up_align = QVector3D::dotProduct(view_dir, QVector3D(0.f, 1.f, 0.f));
        // Avoid gimbal-ish flipping close to poles.
        const bool pitch_would_flip = (up_align > 0.985f && pitch_deg < 0.f) ||
                                      (up_align < -0.985f && pitch_deg > 0.f);

        cam->panAboutViewCenter(yaw_deg);
        if (!pitch_would_flip)
            cam->tiltAboutViewCenter(pitch_deg);
    }

    // ---- Pan: right drag ---------------------------------------------------
    if (state.rightMouseButtonActive && !state.leftMouseButtonActive && !state.middleMouseButtonActive)
    {
        // Translate in camera-local XY (screen plane), moving view centre too.
        cam->translate(QVector3D(-state.rxAxisValue * linearSpeed() * dt,
                                 -state.ryAxisValue * linearSpeed() * dt,
                                  0.f),
                       Qt3DRender::QCamera::TranslateViewCenter);
    }

    // ---- Middle / Left+Right: zoom + roll --------------------------------
    if (zoom_roll_mode)
    {
        // Vertical drag: dolly (clamped around current pivot)
        constexpr float drag_zoom_gain = 2.4f;
        const float drag_step = state.ryAxisValue * lin * drag_zoom_gain;
        dolly_clamped(drag_step);

        // Horizontal drag: roll around view axis
        const float roll_deg = -state.rxAxisValue * look * 0.75f;
        if (std::abs(roll_deg) > 1e-4f)
            cam->roll(roll_deg);
    }

    // ---- Zoom: scroll wheel -----------------------------------------------
    if (qAbs(state.tzAxisValue) > 0.f)
    {
        // Wheel zoom must not depend on frame dt: large dt spikes caused jumps.
        const float dist = (cam->viewCenter() - cam->position()).length();
        const float adaptive = std::max(0.05f, dist * 0.12f);
        const float wheel_step = state.tzAxisValue * adaptive;
        dolly_clamped(wheel_step);
    }
}

// ---------------------------------------------------------------------------
// Constructor – builds the fixed part of the scene graph:
//   camera, lights, robot placeholder box
// ---------------------------------------------------------------------------
Viewer3D::Viewer3D(QWidget* parent, float wall_height)
    : QObject(parent), wall_height_(wall_height)
{
    // ---- Qt3D render window and host widget --------------------------------
    window_    = new Qt3DExtras::Qt3DWindow();
    window_->defaultFrameGraph()->setClearColor(QColor(28, 28, 38));
    // Disable automatic frustum culling: gizmo sub-entities have small bounding
    // volumes and get incorrectly culled when the camera orbits or zooms.
    window_->defaultFrameGraph()->setFrustumCullingEnabled(false);

    // ---- Enable object picking --------------------------------------------
    // Without TrianglePicking the QObjectPicker pressed signal never fires.
    auto* pickSettings = window_->renderSettings()->pickingSettings();
    pickSettings->setPickMethod(Qt3DRender::QPickingSettings::TrianglePicking);
    pickSettings->setPickResultMode(Qt3DRender::QPickingSettings::NearestPriorityPick);
    pickSettings->setFaceOrientationPickingMode(Qt3DRender::QPickingSettings::FrontAndBackFace);

    container_ = QWidget::createWindowContainer(window_, parent);
    container_->setMinimumSize(350, 250);
    container_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container_->setFocusPolicy(Qt::StrongFocus);   // allow keyboard input for camera
    container_->setMouseTracking(true);
    container_->installEventFilter(this);
    // Also filter the window itself: QWidget::createWindowContainer delivers
    // MouseButtonPress directly to the QWindow, not the container widget.
    window_->installEventFilter(this);

    // ---- Pick-result overlay label ----------------------------------------
    pick_label_ = new QLabel(container_);
    pick_label_->setStyleSheet(
        "QLabel { color: white; background-color: rgba(0,0,0,170);"
        " border-radius: 6px; padding: 6px 12px;"
        " font-size: 14px; font-weight: bold; }");
    pick_label_->hide();
    pick_timer_ = new QTimer(this);
    pick_timer_->setSingleShot(true);
    connect(pick_timer_, &QTimer::timeout, pick_label_, &QLabel::hide);

    // ---- Root entity -------------------------------------------------------
    root_entity_ = new Qt3DCore::QEntity();

    // ---- Camera ------------------------------------------------------------
    camera_ = window_->camera();
    // Lower near plane helps avoid gizmo clipping/disappearance when user zooms in.
    camera_->lens()->setPerspectiveProjection(45.f, 16.f / 9.f, 0.001f, 500.f);
    camera_->setPosition(QVector3D(0.f, 15.f, 15.f));
    camera_->setViewCenter(QVector3D(0.f, 0.f, 0.f));
    camera_->setUpVector(QVector3D(0.f, 1.f, 0.f));
    Qt3DRender::QCamera* camera = camera_;  // kept for camCtrl below

    cam_controller_ = new WebotsStyleCameraController(root_entity_);
    cam_controller_->setCamera(camera);
    cam_controller_->setLinearSpeed(20.f);
    cam_controller_->setLookSpeed(180.f);

    // Keep gizmo screen-scale accurate on every camera position change,
    // not only during mouse/wheel events (the camera controller can update
    // the position asynchronously between GUI events).
    connect(camera_, &Qt3DRender::QCamera::positionChanged, this,
            &Viewer3D::update_gizmo_screen_scale);

    // ---- Directional light (sun from upper-left) ---------------------------
    auto* sunEntity = new Qt3DCore::QEntity(root_entity_);
    auto* dirLight  = new Qt3DRender::QDirectionalLight(sunEntity);
    dirLight->setColor(QColor(255, 252, 240));  // warm white
    dirLight->setIntensity(0.55f);              // reduced – was 1.1
    dirLight->setWorldDirection(QVector3D(-0.4f, -1.0f, -0.3f).normalized());
    sunEntity->addComponent(dirLight);

    // ---- Ambient fill (soft warm fill from above) --------------------------
    auto* ambEntity    = new Qt3DCore::QEntity(root_entity_);
    auto* ambLight     = new Qt3DRender::QPointLight(ambEntity);
    ambLight->setColor(QColor(200, 205, 215));
    ambLight->setIntensity(0.25f);              // reduced – was 0.5
    ambLight->setConstantAttenuation(1.f);      // distance attenuation completely off
    ambLight->setLinearAttenuation(0.f);
    ambLight->setQuadraticAttenuation(0.f);
    auto* ambTransform = new Qt3DCore::QTransform(ambEntity);
    ambTransform->setTranslation(QVector3D(0.f, 30.f, 0.f));
    ambEntity->addComponent(ambLight);
    ambEntity->addComponent(ambTransform);

// ---- Robot mesh (meshes/shadow.obj) ------------------------------------
    auto* robotEntity = new Qt3DCore::QEntity(root_entity_);

    auto* robotMesh = new Qt3DRender::QMesh(robotEntity);
    // Absolute path so it resolves correctly regardless of CWD.
    const QString component_root_ctor = QDir(QCoreApplication::applicationDirPath() + "/..").absolutePath();
    robotMesh->setSource(QUrl::fromLocalFile(component_root_ctor + "/meshes/shadow.obj"));

    auto* robotMat = new Qt3DExtras::QPhongMaterial(robotEntity);
    robotMat->setDiffuse(QColor(50, 110, 220));
    robotMat->setAmbient(QColor(20, 45, 95));
    robotMat->setSpecular(QColor(180, 200, 255));
    robotMat->setShininess(60.f);

    robot_transform_ = new Qt3DCore::QTransform(robotEntity);
    // Mesh correction: shadow.obj uses Z-up and faces +Z.
    //   Step 1: -90° around X  → stands upright (fixes "leaning on floor")
    //   Step 2: 180° around Y  → properly aligned based on visual offset
    robot_base_rot_ = QQuaternion::fromAxisAndAngle(0.f, 1.f, 0.f, 180.f)
                    * QQuaternion::fromAxisAndAngle(1.f, 0.f, 0.f, -90.f);
    robot_transform_->setTranslation(QVector3D(0.f, robot_half_h_, 0.f));  // X=0 at start, negated when updated

    robotEntity->addComponent(robotMesh);
    robotEntity->addComponent(robotMat);
    robotEntity->addComponent(robot_transform_);
    attach_picker(robotEntity, "Robot");

    // ---- Pre-allocate lidar point pool ------------------------------------
    init_lidar_pool();

    // ---- Editing gizmo -----------------------------------------------------
    init_gizmo();

    // ---- Commit root entity ------------------------------------------------
    window_->setRootEntity(root_entity_);
}

// ---------------------------------------------------------------------------
// Camera state I/O
// ---------------------------------------------------------------------------
rc::Viewer3D::CameraState rc::Viewer3D::camera_state() const
{
    if (!camera_) return {};
    return { camera_->position(), camera_->viewCenter(), camera_->upVector() };
}

void rc::Viewer3D::set_camera_state(const CameraState& s)
{
    if (!camera_) return;
    camera_->setPosition(s.position);
    camera_->setViewCenter(s.viewCenter);
    camera_->setUpVector(s.upVector);
}

// ---------------------------------------------------------------------------
// Floor plane
// ---------------------------------------------------------------------------
void Viewer3D::build_floor(const std::vector<Eigen::Vector2f>& poly)
{
    if (floor_entity_)
    {
        floor_entity_->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
        delete floor_entity_;
        floor_entity_ = nullptr;
    }
    if (poly.empty()) return;

    // Axis-aligned bounding box of the room polygon
    float min_x = poly[0].x(), max_x = poly[0].x();
    float min_y = poly[0].y(), max_y = poly[0].y();
    for (const auto& v : poly)
    {
        min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
        min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
    }
    const float cx   = (min_x + max_x) * 0.5f;
    const float cy   = (min_y + max_y) * 0.5f;
    const float fw   = (max_x - min_x) + 1.0f;  // slight margin
    const float fh   = (max_y - min_y) + 1.0f;

    floor_entity_ = new Qt3DCore::QEntity(root_entity_);

    auto* floorMesh = new Qt3DExtras::QPlaneMesh();
    floorMesh->setWidth(fw);
    floorMesh->setHeight(fh);
    floorMesh->setMeshResolution(QSize(2, 2));

    // QPlaneMesh lies in the XZ plane, centered at origin – perfect for our mapping.

    auto* floorMat = new Qt3DExtras::QPhongMaterial(floor_entity_);
    floorMat->setDiffuse(QColor(65, 62, 56));   // mid-dark slate
    floorMat->setAmbient(QColor(38, 36, 32));
    floorMat->setSpecular(QColor(0, 0, 0));     // no specular – kills the white hotspot
    floorMat->setShininess(0.f);

    auto* floorXf = new Qt3DCore::QTransform(floor_entity_);
    // Place floor at Y=0, centered on room centroid (X is negated – scene is X-flipped)
    floorXf->setTranslation(QVector3D(-cx, 0.f, cy));

    floor_entity_->addComponent(floorMesh);
    floor_entity_->addComponent(floorMat);
    floor_entity_->addComponent(floorXf);

    attach_picker(floor_entity_, "Floor");
}

// ---------------------------------------------------------------------------
// rebuild_walls  – called whenever room_polygon_ changes
// ---------------------------------------------------------------------------
void Viewer3D::rebuild_walls(const std::vector<Eigen::Vector2f>& poly)
{
    // Delete old wall entities
    for (auto* e : wall_entities_)
    {
        e->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
        delete e;
    }
    wall_entities_.clear();

    if (poly.size() < 2) return;

    build_floor(poly);

    // Re-position camera to frame the room
    const Eigen::Vector2f cen = polygon_centroid(poly);
    float span = 0.f;
    for (const auto& v : poly)
        span = std::max(span, (v - cen).norm());
    const float view_dist = std::max(8.f, span * 1.8f);

    Qt3DRender::QCamera* cam = window_->camera();
    cam->setViewCenter(QVector3D(-cen.x(), 0.f, cen.y()));
    cam->setPosition(QVector3D(-cen.x(), view_dist, cen.y() - view_dist));
    cam->setUpVector(QVector3D(0.f, 1.f, 0.f));

    // One QCuboidMesh per wall segment
    const QColor wall_diffuse(248, 248, 245);   // near-white walls
    const QColor wall_ambient(180, 180, 178);

    for (std::size_t i = 0; i < poly.size(); ++i)
    {
        const Eigen::Vector2f& p1 = poly[i];
        const Eigen::Vector2f& p2 = poly[(i + 1) % poly.size()];

        const Eigen::Vector2f mid = (p1 + p2) * 0.5f;
        const float        length = (p2 - p1).norm();
        if (length < 1e-3f) continue;

        const float angle_rad = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());

        auto* wallE = new Qt3DCore::QEntity(root_entity_);

        // Geometry: xExtent = wall length (default QCuboidMesh is aligned to X)
        auto* mesh = new Qt3DExtras::QCuboidMesh();
        mesh->setXExtent(length);
        mesh->setYExtent(wall_height_);
        mesh->setZExtent(0.12f);           // wall thickness (m)

        auto* mat = new Qt3DExtras::QPhongMaterial(wallE);
        mat->setDiffuse(wall_diffuse);
        mat->setAmbient(wall_ambient);
        mat->setShininess(2.f);   // matte white

        auto* tf = new Qt3DCore::QTransform(wallE);
        // X is negated globally: place at -mid.x().
        // With X flipped the wall direction angle negates too, so rotation becomes +angle_rad.
        tf->setTranslation(QVector3D(-mid.x(), wall_height_ * 0.5f, mid.y()));
        tf->setRotationY(qRadiansToDegrees(angle_rad));

        wallE->addComponent(mesh);
        wallE->addComponent(mat);
        wallE->addComponent(tf);
        attach_picker(wallE, QStringLiteral("Wall %1").arg(i+1));

        wall_entities_.push_back(wallE);
    }
}

// ---------------------------------------------------------------------------
// update_robot_pose  – called every visualization frame (~10 Hz)
// ---------------------------------------------------------------------------
void Viewer3D::update_robot_pose(float x, float y, float theta_rad)
{
    if (!robot_transform_) return;
    robot_transform_->setTranslation(QVector3D(-x, robot_half_h_, y));
    // Heading in room frame: positive theta_rad = CCW around Z in 2D.
    const QQuaternion heading = QQuaternion::fromAxisAndAngle(0.f, 1.f, 0.f,
                                    qRadiansToDegrees(theta_rad));
    // Apply base correction first, then heading on top.
    robot_transform_->setRotation(heading * robot_base_rot_);
}

// ---------------------------------------------------------------------------
// update_obstacles  – called after temp_obstacles_ list changes
// ---------------------------------------------------------------------------
void Viewer3D::update_obstacles(const std::vector<ObstacleItem>& obstacles)
{
    // Remove all previous obstacle entities
    for (auto* e : obstacle_entities_)
    {
        e->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
        delete e;
    }
    obstacle_entities_.clear();

    static const QColor obs_diffuse(240, 110, 25);
    static const QColor obs_ambient(100, 44, 10);

    for (const auto& item : obstacles)
    {
        const auto& poly = item.vertices;
        if (poly.size() < 3) continue;

        // AABB approximation of the obstacle polygon
        float min_x = poly[0].x(), max_x = poly[0].x();
        float min_y = poly[0].y(), max_y = poly[0].y();
        for (const auto& v : poly)
        {
            min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
            min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
        }
        const float cx    = (min_x + max_x) * 0.5f;
        const float cy    = (min_y + max_y) * 0.5f;
        const float ext_x = std::max(0.15f, max_x - min_x);
        const float ext_y = std::max(0.15f, max_y - min_y);
        // Use lidar-estimated height when available; fall back to 85 % of wall height
        const float h = (item.height > 0.05f) ? item.height : (wall_height_ * 0.85f);

        auto* obsE = new Qt3DCore::QEntity(root_entity_);

        auto* mesh = new Qt3DExtras::QCuboidMesh();
        mesh->setXExtent(ext_x);
        mesh->setYExtent(h);
        mesh->setZExtent(ext_y);

        auto* mat = new Qt3DExtras::QPhongMaterial(obsE);
        mat->setDiffuse(obs_diffuse);
        mat->setAmbient(obs_ambient);
        mat->setSpecular(QColor(255, 200, 150));
        mat->setShininess(40.f);

        auto* tf = new Qt3DCore::QTransform(obsE);
        tf->setTranslation(QVector3D(-cx, h * 0.5f, cy));

        obsE->addComponent(mesh);
        obsE->addComponent(mat);
        obsE->addComponent(tf);
        attach_picker(obsE, QString("Obstacle %1 (h=%2 m)").arg(obstacle_entities_.size() + 1).arg(h, 0, 'f', 2));

        obstacle_entities_.push_back(obsE);
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
Eigen::Vector2f Viewer3D::polygon_centroid(const std::vector<Eigen::Vector2f>& poly)
{
    if (poly.empty()) return Eigen::Vector2f::Zero();
    Eigen::Vector2f sum = Eigen::Vector2f::Zero();
    for (const auto& v : poly) sum += v;
    return sum / static_cast<float>(poly.size());
}

// ---------------------------------------------------------------------------
// update_furniture  – one (or two) mesh entities per furniture polygon
// ---------------------------------------------------------------------------
void Viewer3D::update_furniture(const std::vector<FurnitureItem>& items)
{
    // Remove all previous furniture entities
    for (auto* e : furniture_entities_)
    {
        e->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
        delete e;
    }
    furniture_entities_.clear();
    furniture_centers_world_.clear();
    furniture_groups_.clear();

    auto register_group_part = [&](const std::string& group_key,
                                   const Eigen::Vector2f& centroid,
                                   float yaw_rad,
                                   Qt3DCore::QTransform* tf) -> void
    {
        auto& g = furniture_groups_[group_key];
        if (g.transforms.empty())
        {
            g.center_world = QVector3D(-centroid.x(), 0.f, centroid.y());
            g.yaw_rad = yaw_rad;
        }
        g.transforms.push_back(tf);
        g.local_offsets.push_back(tf->translation() - g.center_world);
    };

    // ----------------------------------------------------------------
    // Procedural cuboid part helper.  All objects are built from one or
    // more cuboid parts positioned relative to the object centroid using
    // the object's yaw.
    // ----------------------------------------------------------------
    auto make_cuboid_part = [&](const QColor&        diffuse,
                                const QColor&        ambient,
                                float                shininess,
                                const Eigen::Vector2f& centroid,
                                float                yaw_rad,
                                float                local_x,     // local offset along width axis  (local X)
                                float                local_y,     // local offset along depth axis   (local Y)
                                float                size_x,      // cuboid width  (along local X)
                                float                size_y,      // cuboid depth  (along local Y)
                                float                size_h,      // cuboid height (world Y-up)
                                float                center_h,    // vertical centre of this part
                                const QString&       pick_name,
                                const std::string&   group_key) -> void
    {
        // Standard 2D rotation: world = center + R(yaw) * local
        //   R(yaw) = [[cos, -sin], [sin, cos]]
        // Same convention as object_footprints.h.
        const float c = std::cos(yaw_rad);
        const float s = std::sin(yaw_rad);
        const float wx = centroid.x() + c * local_x - s * local_y;
        const float wy = centroid.y() + s * local_x + c * local_y;

        // Qt3D Y-rotation: width axis (local X) in Qt3D = (-cos θ, 0, sin θ).
        // Ry(α) maps (1,0,0)→(cos α, 0, -sin α).
        // cos α = -cos θ, sin α = -sin θ  →  α = π + θ.
        const QQuaternion yaw_rot = QQuaternion::fromAxisAndAngle(
            0.f, 1.f, 0.f,
            qRadiansToDegrees(static_cast<float>(M_PI) + yaw_rad));

        auto* e    = new Qt3DCore::QEntity(root_entity_);
        auto* mesh = new Qt3DExtras::QCuboidMesh(e);
        mesh->setXExtent(std::max(0.03f, size_x));
        mesh->setYExtent(std::max(0.03f, size_h));
        mesh->setZExtent(std::max(0.03f, size_y));

        auto* mat = new Qt3DExtras::QPhongMaterial(e);
        mat->setDiffuse(diffuse);
        mat->setAmbient(ambient);
        mat->setShininess(shininess);

        auto* tf = new Qt3DCore::QTransform(e);
        tf->setTranslation(QVector3D(-wx, center_h, wy));
        tf->setRotation(yaw_rot);

        e->addComponent(mesh);
        e->addComponent(mat);
        e->addComponent(tf);
        attach_picker(e, pick_name);
        furniture_entities_.push_back(e);
        register_group_part(group_key, centroid, yaw_rad, tf);
    };

    // ----------------------------------------------------------------
    // Procedural object builders
    // ----------------------------------------------------------------

    // Table: tabletop + 4 legs
    auto make_table = [&](const Eigen::Vector2f& cen, float yaw,
                          float w, float d, float h,
                          const QString& pn, const std::string& gk)
    {
        const float top_thick = std::max(0.03f, 0.08f * h);
        const float leg_thick = std::max(0.03f, 0.12f * std::min(w, d));
        const float leg_h     = std::max(0.05f, h - top_thick);
        const float leg_dx    = std::max(0.f, 0.5f * w - 0.5f * leg_thick);
        const float leg_dy    = std::max(0.f, 0.5f * d - 0.5f * leg_thick);
        // Tabletop
        make_cuboid_part(QColor(160, 100, 55), QColor(70, 44, 24), 20.f,
                         cen, yaw, 0.f, 0.f, w, d, top_thick,
                         leg_h + 0.5f * top_thick, pn, gk);
        // Four legs
        for (float sx : {1.f, -1.f})
            for (float sy : {1.f, -1.f})
                make_cuboid_part(QColor(140, 88, 48), QColor(60, 38, 18), 15.f,
                                 cen, yaw, sx * leg_dx, sy * leg_dy,
                                 leg_thick, leg_thick, leg_h,
                                 0.5f * leg_h, pn, gk);
    };

    // Chair: seat + backrest + 4 legs
    auto make_chair = [&](const Eigen::Vector2f& cen, float yaw,
                          float w, float d, float h,
                          const QString& pn, const std::string& gk)
    {
        const float seat_thick = std::max(0.03f, 0.06f * h);
        const float seat_h     = 0.48f * h;          // seat height fraction
        const float leg_thick  = std::max(0.03f, 0.10f * std::min(w, d));
        const float leg_dx     = std::max(0.f, 0.5f * w - 0.5f * leg_thick);
        const float leg_dy     = std::max(0.f, 0.5f * d - 0.5f * leg_thick);
        const float back_thick = std::max(0.03f, 0.06f * d);
        const float back_h     = h - seat_h - seat_thick;
        // Seat
        make_cuboid_part(QColor(110, 85, 60), QColor(50, 35, 20), 18.f,
                         cen, yaw, 0.f, 0.f, w, d, seat_thick,
                         seat_h + 0.5f * seat_thick, pn, gk);
        // Four legs
        for (float sx : {1.f, -1.f})
            for (float sy : {1.f, -1.f})
                make_cuboid_part(QColor(100, 75, 50), QColor(45, 30, 18), 15.f,
                                 cen, yaw, sx * leg_dx, sy * leg_dy,
                                 leg_thick, leg_thick, seat_h,
                                 0.5f * seat_h, pn, gk);
        // Backrest (offset along local depth axis, behind the seat)
        make_cuboid_part(QColor(120, 90, 62), QColor(55, 40, 25), 18.f,
                         cen, yaw, 0.f, -(0.5f * d - 0.5f * back_thick),
                         w, back_thick, back_h,
                         seat_h + seat_thick + 0.5f * back_h, pn, gk);
    };

    // Bench (park bench): seat plank + backrest + stout legs
    auto make_bench = [&](const Eigen::Vector2f& cen, float yaw,
                          float w, float d, float h,
                          const QString& pn, const std::string& gk)
    {
        const float seat_thick = std::max(0.04f, 0.08f * h);
        const float seat_h     = 0.45f * h;           // seat height
        const float leg_thick  = std::max(0.04f, 0.12f * std::min(w, d));
        const float leg_dx     = std::max(0.f, 0.42f * w);  // spread along width
        const float back_thick = std::max(0.04f, 0.08f * d);
        const float back_h     = std::max(0.08f, h - seat_h - seat_thick);
        // Seat plank (wider than deep, like a park bench)
        make_cuboid_part(QColor(230, 140, 30), QColor(180, 100, 15), 15.f,
                         cen, yaw, 0.f, 0.f, w, d, seat_thick,
                         seat_h + 0.5f * seat_thick, pn, gk);
        // Two pairs of stout legs (no rear legs separate — bench style)
        for (float sx : {1.f, -1.f})
            make_cuboid_part(QColor(190, 110, 20), QColor(140, 80, 10), 12.f,
                             cen, yaw, sx * leg_dx, 0.f,
                             leg_thick, d * 0.85f, seat_h,
                             0.5f * seat_h, pn, gk);
        // Backrest (tall thin board at rear edge, -Y local)
        make_cuboid_part(QColor(230, 140, 30), QColor(180, 100, 15), 15.f,
                         cen, yaw, 0.f, -(0.5f * d - 0.5f * back_thick),
                         w, back_thick, back_h,
                         seat_h + seat_thick + 0.5f * back_h, pn, gk);
    };

    // Pot / planter : lower box + upper crown
    auto make_pot = [&](const Eigen::Vector2f& cen, float yaw,
                        float w, float d, float h,
                        const QString& pn, const std::string& gk)
    {
        // Truncated cone: smaller radius at base, larger at top.
        const float top_r    = std::max(w, d) * 0.5f;
        const float bottom_r = top_r * 0.65f;   // base is ~65% of top
        const float pot_h    = 0.45f * h;
        const float crown_h  = h - pot_h;

        // World position: Qt3D uses X-right, Y-up, Z-toward-camera.
        const float wx = -cen.x();
        const float wz =  cen.y();

        // ---- Invisible bounding cuboid for reliable picking ----
        // QConeMesh / QSphereMesh have unreliable triangle picking in Qt3D.
        {
            const float crown_r = top_r * 1.10f;
            const float bbox_w  = 2.f * crown_r;
            auto* e    = new Qt3DCore::QEntity(root_entity_);
            auto* mesh = new Qt3DExtras::QCuboidMesh(e);
            mesh->setXExtent(bbox_w);
            mesh->setYExtent(h);
            mesh->setZExtent(bbox_w);

            auto* mat = new Qt3DExtras::QPhongAlphaMaterial(e);
            mat->setAlpha(0.f);

            auto* tf = new Qt3DCore::QTransform(e);
            tf->setTranslation(QVector3D(wx, 0.5f * h, wz));

            e->addComponent(mesh);
            e->addComponent(mat);
            e->addComponent(tf);
            attach_picker(e, pn);
            furniture_entities_.push_back(e);
        }

        // ---- Pot body: truncated cone (QConeMesh with topRadius > bottomRadius) ----
        {
            auto* e    = new Qt3DCore::QEntity(root_entity_);
            auto* cone  = new Qt3DExtras::QConeMesh(e);
            cone->setLength(pot_h);
            cone->setBottomRadius(bottom_r);  // narrow base
            cone->setTopRadius(top_r);        // wider top opening
            cone->setRings(1);
            cone->setSlices(24);

            auto* mat = new Qt3DExtras::QPhongMaterial(e);
            mat->setDiffuse(QColor(195, 155, 95));
            mat->setAmbient(QColor(90, 70, 40));
            mat->setShininess(10.f);

            auto* tf = new Qt3DCore::QTransform(e);
            tf->setTranslation(QVector3D(wx, 0.5f * pot_h, wz));

            e->addComponent(cone);
            e->addComponent(mat);
            e->addComponent(tf);
            furniture_entities_.push_back(e);
            register_group_part(gk, cen, yaw, tf);
        }
        // ---- Crown: green sphere on top ----
        {
            auto* e    = new Qt3DCore::QEntity(root_entity_);
            auto* mesh = new Qt3DExtras::QSphereMesh(e);
            const float crown_r = top_r * 1.10f;
            mesh->setRadius(crown_r);
            mesh->setRings(12);
            mesh->setSlices(16);
            // Squash vertically: scale Y so it becomes an oblate sphere.
            const float sy = crown_h / (2.0f * crown_r);

            auto* mat = new Qt3DExtras::QPhongMaterial(e);
            mat->setDiffuse(QColor(55, 130, 45));
            mat->setAmbient(QColor(25, 60, 20));
            mat->setShininess(5.f);

            auto* tf = new Qt3DCore::QTransform(e);
            tf->setTranslation(QVector3D(wx, pot_h + 0.5f * crown_h, wz));
            tf->setScale3D(QVector3D(1.f, sy, 1.f));

            e->addComponent(mesh);
            e->addComponent(mat);
            e->addComponent(tf);
            furniture_entities_.push_back(e);
            register_group_part(gk, cen, yaw, tf);
        }
    };

    // Monitor / screen : thin panel on a stand
    auto make_monitor = [&](const Eigen::Vector2f& cen, float yaw,
                            float w, float d, float h,
                            const QString& pn, const std::string& gk)
    {
        const float stand_h = 0.45f * h;
        const float panel_h = h - stand_h;
        const float stand_thick = std::min(0.08f, 0.3f * d);
        // Stand post
        make_cuboid_part(QColor(60, 60, 65), QColor(30, 30, 32), 50.f,
                         cen, yaw, 0.f, 0.f, stand_thick, stand_thick, stand_h,
                         0.5f * stand_h, pn, gk);
        // Screen panel
        make_cuboid_part(QColor(20, 22, 30), QColor(10, 11, 15), 80.f,
                         cen, yaw, 0.f, 0.f, w, 0.04f, panel_h,
                         stand_h + 0.5f * panel_h, pn, gk);
    };

    // Cabinet / shelf : single box
    auto make_cabinet = [&](const Eigen::Vector2f& cen, float yaw,
                            float w, float d, float h,
                            const QString& pn, const std::string& gk)
    {
        make_cuboid_part(QColor(180, 200, 220), QColor(70, 80, 90), 40.f,
                         cen, yaw, 0.f, 0.f, w, d, h, 0.5f * h, pn, gk);
    };

    // Generic fallback : simple box
    auto make_generic = [&](const Eigen::Vector2f& cen, float yaw,
                            float w, float d, float h,
                            const QString& pn, const std::string& gk)
    {
        make_cuboid_part(QColor(150, 150, 160), QColor(70, 70, 75), 20.f,
                         cen, yaw, 0.f, 0.f, w, d, h, 0.5f * h, pn, gk);
    };

    // ----------------------------------------------------------------
    // Dispatch each item to the appropriate procedural builder
    // ----------------------------------------------------------------
    for (const auto& item : items)
    {
        const QString ql = QString::fromStdString(item.label).toLower();
        const Eigen::Vector2f& cen  = item.centroid;
        const float w = std::max(0.08f, item.size.x());
        const float d = std::max(0.08f, item.size.y());
        const float h = std::max(0.20f, item.height);
        const QString pn = QString::fromStdString(item.label);
        const std::string& gk = item.label;

        if (ql.contains("mesa") || ql.contains("table"))
            make_table(cen, item.yaw_rad, w, d, h, pn, gk);
        else if (ql.contains("silla") || ql.contains("chair"))
            make_chair(cen, item.yaw_rad, w, d, h, pn, gk);
        else if (ql.contains("banco") || ql.contains("bench"))
            make_bench(cen, item.yaw_rad, w, d, h, pn, gk);
        else if (ql.contains("maceta") || ql.contains("plant") || ql.contains("pot"))
            make_pot(cen, item.yaw_rad, w, d, h, pn, gk);
        else if (ql.contains("monitor") || ql.contains("pantalla") || ql.contains("screen") ||
                 ql.contains("ordenador") || ql.contains("computer"))
            make_monitor(cen, item.yaw_rad, w, d, h, pn, gk);
        else if (ql.contains("vitrina") || ql.contains("cabinet") || ql.contains("shelf"))
            make_cabinet(cen, item.yaw_rad, w, d, h, pn, gk);
        else
            make_generic(cen, item.yaw_rad, w, d, h, pn, gk);

        furniture_centers_world_[item.label] = QVector3D(-cen.x(), 0.f, cen.y());

        qInfo() << "[Viewer3D] Furniture:" << pn
                << "at (" << cen.x() << "," << cen.y() << ")"
                << "size (" << w << "x" << d << "x" << h << ")";
    }

    if (!selected_object_name_.isEmpty())
        show_gizmo_for_object(selected_object_name_);
}

void Viewer3D::init_gizmo()
{
    gizmo_root_ = new Qt3DCore::QEntity(root_entity_);
    gizmo_tf_ = new Qt3DCore::QTransform(gizmo_root_);
    gizmo_root_->addComponent(gizmo_tf_);

    auto force_gizmo_on_top = [](Qt3DExtras::QPhongMaterial* mat)
    {
        if (!mat || !mat->effect()) return;
        auto* eff = mat->effect();
        const auto techniques = eff->techniques();
        for (Qt3DRender::QTechnique* tech : techniques)
        {
            if (!tech) continue;
            const auto passes = tech->renderPasses();
            for (Qt3DRender::QRenderPass* pass : passes)
            {
                if (!pass) continue;
                auto* depth_test = new Qt3DRender::QDepthTest(pass);
                depth_test->setDepthFunction(Qt3DRender::QDepthTest::Always);
                pass->addRenderState(depth_test);
            }
        }
    };

    auto make_axis = [&](const QVector3D& axis_dir,
                         const QColor& color,
                         const QString& pick_name)
    {
        // Requested by user: make straight arrows 25% shorter.
        const float axis_end = gizmo_axis_len_ * 0.69f;
        const float shaft_len = std::max(0.20f, axis_end);
        const float shaft_r = 0.022f;
        const float cone_len = 0.15f;
        const float cone_bottom_r = 0.062f;

        auto* e = new Qt3DCore::QEntity(gizmo_root_);
        auto* mesh = new Qt3DExtras::QCylinderMesh(e);
        mesh->setRadius(shaft_r);
        mesh->setLength(shaft_len);
        mesh->setRings(18);
        mesh->setSlices(36);

        auto* mat = new Qt3DExtras::QPhongMaterial(e);
        mat->setDiffuse(color.lighter(110));
        mat->setAmbient(color.darker(115));
        mat->setShininess(0.f);
        force_gizmo_on_top(mat);

        auto* tf = new Qt3DCore::QTransform(e);
        const QVector3D dir_n = axis_dir.normalized();
        // QCylinderMesh axis is Y; rotate Y to axis direction.
        tf->setRotation(QQuaternion::rotationTo(QVector3D(0.f, 1.f, 0.f), dir_n));
        tf->setTranslation(axis_dir * (shaft_len * 0.5f));

        e->addComponent(mesh);
        e->addComponent(mat);
        e->addComponent(tf);
        attach_picker(e, pick_name);

        // Last 15% of the straight arrow: thicker segment to make grabbing easier.
        {
            const float grab_len = std::max(0.08f, axis_end * 0.15f);
            auto* grab_e = new Qt3DCore::QEntity(gizmo_root_);
            auto* grab_mesh = new Qt3DExtras::QCylinderMesh(grab_e);
            grab_mesh->setRadius(shaft_r * 1.35f);
            grab_mesh->setLength(grab_len);
            grab_mesh->setRings(14);
            grab_mesh->setSlices(28);

            auto* grab_tf = new Qt3DCore::QTransform(grab_e);
            grab_tf->setRotation(QQuaternion::rotationTo(QVector3D(0.f, 1.f, 0.f), dir_n));
            grab_tf->setTranslation(axis_dir * (axis_end - 0.5f * grab_len));

            grab_e->addComponent(grab_mesh);
            grab_e->addComponent(mat);
            grab_e->addComponent(grab_tf);
            attach_picker(grab_e, pick_name);
        }

        // Conical tip at the end of the straight arrow.
        auto* tip_e = new Qt3DCore::QEntity(gizmo_root_);
        auto* cone = new Qt3DExtras::QConeMesh(tip_e);
        cone->setLength(cone_len);
        cone->setTopRadius(0.0f);
        cone->setBottomRadius(cone_bottom_r);
        cone->setRings(16);
        cone->setSlices(20);

        auto* tip_tf = new Qt3DCore::QTransform(tip_e);
        // QConeMesh axis is Y; rotate Y to axis direction.
        tip_tf->setRotation(QQuaternion::rotationTo(QVector3D(0.f, 1.f, 0.f), dir_n));
        tip_tf->setTranslation(axis_dir * (axis_end + 0.5f * cone_len));

        tip_e->addComponent(cone);
        tip_e->addComponent(mat);
        tip_e->addComponent(tip_tf);
        attach_picker(tip_e, pick_name);
    };

    auto make_arc = [&](const QVector3D& euler_deg,
                        const QVector3D& axis_tip,
                        const QColor& color,
                        const QString& pick_name)
    {
        auto* arc_root = new Qt3DCore::QEntity(gizmo_root_);

        auto* mat = new Qt3DExtras::QPhongMaterial(arc_root);
        mat->setDiffuse(color);
        mat->setAmbient(color.darker(180));
        mat->setShininess(6.f);
        force_gizmo_on_top(mat);

        auto* tf = new Qt3DCore::QTransform(arc_root);
        const QQuaternion rot = QQuaternion::fromEulerAngles(euler_deg);
        tf->setTranslation(axis_tip);
        tf->setRotation(rot);
        arc_root->addComponent(tf);

        const float radius = gizmo_axis_len_ * 0.085f;
        const float cyl_r = 0.020f;
        constexpr int segments = 72;
        constexpr float start_deg = -135.f;
        constexpr float span_deg = 270.f;

        for (int i = 0; i < segments; ++i)
        {
            const float a0 = qDegreesToRadians(start_deg + span_deg * (static_cast<float>(i) / static_cast<float>(segments)));
            const float a1 = qDegreesToRadians(start_deg + span_deg * (static_cast<float>(i + 1) / static_cast<float>(segments)));

            const QVector3D p0(radius * std::cos(a0), 0.f, radius * std::sin(a0));
            const QVector3D p1(radius * std::cos(a1), 0.f, radius * std::sin(a1));
            const QVector3D mid = 0.5f * (p0 + p1);
            const QVector3D chord = p1 - p0;
            // Slight overlap between consecutive segments removes visible seams.
            const float chord_len = std::max(0.02f, chord.length() * 1.08f);

            auto* seg_e = new Qt3DCore::QEntity(arc_root);
            auto* seg_mesh = new Qt3DExtras::QCylinderMesh(seg_e);
            seg_mesh->setRadius(cyl_r);
            seg_mesh->setLength(chord_len);
            seg_mesh->setRings(12);
            seg_mesh->setSlices(24);

            auto* seg_tf = new Qt3DCore::QTransform(seg_e);
            seg_tf->setTranslation(mid);
            // Cylinder local axis is Y. Rotate Y to chord direction.
            seg_tf->setRotation(QQuaternion::rotationTo(QVector3D(0.f, 1.f, 0.f), chord.normalized()));

            seg_e->addComponent(seg_mesh);
            seg_e->addComponent(mat);
            seg_e->addComponent(seg_tf);
            attach_picker(seg_e, pick_name);
        }

        // Conical tip at the end of the curved arrow.
        const float end_deg = start_deg + span_deg;
        const float end_rad = qDegreesToRadians(end_deg);
        const QVector3D end_p(radius * std::cos(end_rad), 0.f, radius * std::sin(end_rad));
        const QVector3D tangent(-std::sin(end_rad), 0.f, std::cos(end_rad));

        auto* tip_e = new Qt3DCore::QEntity(arc_root);
        auto* cone = new Qt3DExtras::QConeMesh(tip_e);
        cone->setLength(0.12f);
        cone->setTopRadius(0.0f);
        cone->setBottomRadius(0.050f);
        cone->setRings(14);
        cone->setSlices(18);

        auto* tip_tf = new Qt3DCore::QTransform(tip_e);
        tip_tf->setRotation(QQuaternion::rotationTo(QVector3D(0.f, 1.f, 0.f), tangent.normalized()));
        tip_tf->setTranslation(end_p + tangent.normalized() * 0.06f);

        tip_e->addComponent(cone);
        tip_e->addComponent(mat);
        tip_e->addComponent(tip_tf);
        attach_picker(tip_e, pick_name);
    };

    // Requested mapping: X red, Y green, Z blue (vertical).
    make_axis(QVector3D(1.f, 0.f, 0.f), QColor(225, 70, 70), "__GIZMO_MOVE_X");
    make_axis(QVector3D(0.f, 0.f, 1.f), QColor(70, 220, 95), "__GIZMO_MOVE_Y");
    make_axis(QVector3D(0.f, 1.f, 0.f), QColor(70, 120, 235), "__GIZMO_MOVE_Z");

    // Small center marker like Webots-style gizmos.
    {
        auto* center_e = new Qt3DCore::QEntity(gizmo_root_);
        auto* center_mesh = new Qt3DExtras::QSphereMesh(center_e);
        center_mesh->setRadius(0.055f);
        center_mesh->setRings(14);
        center_mesh->setSlices(18);
        auto* center_mat = new Qt3DExtras::QPhongMaterial(center_e);
        center_mat->setDiffuse(QColor(220, 220, 230));
        center_mat->setAmbient(QColor(110, 110, 120));
        center_mat->setShininess(0.f);
        force_gizmo_on_top(center_mat);
        center_e->addComponent(center_mesh);
        center_e->addComponent(center_mat);
    }

    // Requested by user: 270-degree curved handles placed near each axis tip.
    const float arc_offset = gizmo_axis_len_ * 0.69f * 0.78f;
    make_arc(QVector3D(0.f, 0.f, 90.f), QVector3D(arc_offset, 0.f, 0.f), QColor(225, 90, 90), "__GIZMO_ROT_X");
    // Y is green horizontal (scene Z). Z is blue vertical (scene Y).
    make_arc(QVector3D(90.f, 0.f, 0.f), QVector3D(0.f, 0.f, arc_offset), QColor(95, 230, 95), "__GIZMO_ROT_Y");
    make_arc(QVector3D(0.f, 0.f, 0.f), QVector3D(0.f, arc_offset, 0.f), QColor(90, 140, 240), "__GIZMO_ROT_Z");

    hide_gizmo();
}

bool Viewer3D::translate_furniture_object(const QString& name, float dx_room, float dy_room)
{
    if (name.trimmed().isEmpty() || furniture_groups_.empty())
        return false;

    auto normalize = [](QString s)
    {
        s = s.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s.toLower();
    };

    const QString target = normalize(name);
    auto it = furniture_groups_.end();
    for (auto fit = furniture_groups_.begin(); fit != furniture_groups_.end(); ++fit)
    {
        const QString key = normalize(QString::fromStdString(fit->first));
        if (key == target || key.contains(target) || target.contains(key))
        {
            it = fit;
            break;
        }
    }
    if (it == furniture_groups_.end())
        return false;

    QVector3D d(-dx_room, 0.f, dy_room);
    auto& g = it->second;
    g.center_world += d;
    for (std::size_t i = 0; i < g.transforms.size() && i < g.local_offsets.size(); ++i)
        g.transforms[i]->setTranslation(g.center_world + g.local_offsets[i]);

    furniture_centers_world_[it->first] = g.center_world;
    return true;
}

bool Viewer3D::set_furniture_absolute_center(const QString& name, float room_x, float room_y)
{
    // Find the group first to read the current world position.
    auto normalize = [](QString s)
    {
        s = s.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s.toLower();
    };

    const QString target = normalize(name);
    for (auto& [key_s, g] : furniture_groups_)
    {
        if (normalize(QString::fromStdString(key_s)) != target &&
            !normalize(QString::fromStdString(key_s)).contains(target) &&
            !target.contains(normalize(QString::fromStdString(key_s))))
            continue;

        // room_x → scene -x,  room_y → scene +z
        // current: g.center_world.x() == -current_room_x,  g.center_world.z() == current_room_y
        const float dx_room = -room_x - g.center_world.x();   // target_scene_x - current_scene_x, in room units (sign already handles flip)
        const float dy_room =  room_y - g.center_world.z();
        return translate_furniture_object(name, dx_room, dy_room);
    }
    return false;
}

bool Viewer3D::rotate_furniture_object(const QString& name, float angle_rad, const QVector3D& axis)
{
    if (name.trimmed().isEmpty() || furniture_groups_.empty() || std::abs(angle_rad) < 1e-7f)
        return false;

    auto normalize = [](QString s)
    {
        s = s.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s.toLower();
    };

    const QString target = normalize(name);
    auto it = furniture_groups_.end();
    for (auto fit = furniture_groups_.begin(); fit != furniture_groups_.end(); ++fit)
    {
        const QString key = normalize(QString::fromStdString(fit->first));
        if (key == target || key.contains(target) || target.contains(key))
        {
            it = fit;
            break;
        }
    }
    if (it == furniture_groups_.end())
        return false;

    auto& g = it->second;
    const QQuaternion d_rot = QQuaternion::fromAxisAndAngle(axis, qRadiansToDegrees(angle_rad));
    for (std::size_t i = 0; i < g.transforms.size() && i < g.local_offsets.size(); ++i)
    {
        g.local_offsets[i] = d_rot.rotatedVector(g.local_offsets[i]);
        g.transforms[i]->setTranslation(g.center_world + g.local_offsets[i]);
        g.transforms[i]->setRotation(d_rot * g.transforms[i]->rotation());
    }

    // Track yaw only for vertical-axis rotations.
    if (std::abs(axis.y()) > 0.5f)
        g.yaw_rad += angle_rad;
    return true;
}

bool Viewer3D::vertical_move_furniture_object(const QString& name, float dy_world)
{
    if (name.trimmed().isEmpty() || furniture_groups_.empty() || std::abs(dy_world) < 1e-7f)
        return false;

    auto normalize = [](QString s)
    {
        s = s.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s.toLower();
    };

    const QString target = normalize(name);
    auto it = furniture_groups_.end();
    for (auto fit = furniture_groups_.begin(); fit != furniture_groups_.end(); ++fit)
    {
        const QString key = normalize(QString::fromStdString(fit->first));
        if (key == target || key.contains(target) || target.contains(key))
        {
            it = fit;
            break;
        }
    }
    if (it == furniture_groups_.end())
        return false;

    const QVector3D d(0.f, dy_world, 0.f);
    auto& g = it->second;
    g.center_world += d;
    for (std::size_t i = 0; i < g.transforms.size() && i < g.local_offsets.size(); ++i)
        g.transforms[i]->setTranslation(g.center_world + g.local_offsets[i]);

    furniture_centers_world_[it->first] = g.center_world;
    return true;
}

void Viewer3D::update_gizmo_screen_scale()
{
    if (!gizmo_root_ || !gizmo_tf_ || !camera_ || !gizmo_root_->isEnabled())
        return;

    const float dist = (camera_->position() - gizmo_tf_->translation()).length();
    // Keep stable on-screen size without exploding while moving the gizmo.
    const float scale = std::clamp(dist / 13.0f, 1.15f, 2.40f);
    gizmo_tf_->setScale(scale);
}

void Viewer3D::show_gizmo_for_object(const QString& name)
{
    if (name.isEmpty() || !gizmo_root_ || !gizmo_tf_)
    {
        // Do not auto-hide during transient invalid requests if a valid selection exists.
        if (selected_object_name_.isEmpty())
            hide_gizmo();
        return;
    }

    auto normalize = [](const QString& raw) -> QString
    {
        QString s = raw.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s;
    };

    const QString qname = name.trimmed();
    const QString qbase = normalize(qname);
    // Store requested name first so stale previous selection is not reused.
    selected_object_name_ = qbase;

    auto it = furniture_centers_world_.find(qname.toStdString());
    if (it == furniture_centers_world_.end())
        it = furniture_centers_world_.find(qbase.toStdString());

    if (it == furniture_centers_world_.end())
    {
        // Case-insensitive fallback and relaxed matching for picked aliases.
        const QString target = qbase.toLower();
        for (auto fit = furniture_centers_world_.begin(); fit != furniture_centers_world_.end(); ++fit)
        {
            const QString key = QString::fromStdString(fit->first);
            const QString key_base = normalize(key).toLower();
            if (key_base == target || key.toLower().contains(target) || target.contains(key_base))
            {
                it = fit;
                break;
            }
        }
    }

    if (it == furniture_centers_world_.end())
    {
        // Keep current gizmo if lookup misses transiently (e.g. asynchronous refresh).
        if (gizmo_root_ && gizmo_root_->isEnabled() && !selected_object_name_.isEmpty())
            return;
        hide_gizmo();
        return;
    }

    gizmo_tf_->setTranslation(it->second + QVector3D(0.f, 0.85f, 0.f));
    // Match the default selection size to the effective interaction size.
    gizmo_tf_->setScale(1.15f);
    gizmo_root_->setEnabled(true);
}

void Viewer3D::set_selected_object_for_gizmo(const QString& name)
{
    show_gizmo_for_object(name);
}

void Viewer3D::clear_selected_object_for_gizmo()
{
    selected_object_name_.clear();
    hide_gizmo();
}

void Viewer3D::hide_gizmo()
{
    // Guard against unintended hide calls during camera navigation/refresh.
    // Explicit deselection clears selected_object_name_ first and will still hide.
    if (!selected_object_name_.isEmpty())
        return;
    if (gizmo_root_) gizmo_root_->setEnabled(false);
    end_gizmo_drag();
}

void Viewer3D::begin_gizmo_drag(GizmoMode mode)
{
    gizmo_mode_ = mode;
    gizmo_drag_active_ = (mode != GizmoMode::None);
    if (cam_controller_ && gizmo_drag_active_)
        cam_controller_->setEnabled(false);
    if (container_ && gizmo_drag_active_)
        container_->grabMouse();
}

void Viewer3D::end_gizmo_drag()
{
    const bool was_active = gizmo_drag_active_;
    const QString finished_name = selected_object_name_;
    gizmo_drag_active_ = false;
    gizmo_mode_ = GizmoMode::None;
    if (cam_controller_)
        cam_controller_->setEnabled(true);
    if (container_)
        container_->releaseMouse();
    if (was_active && !finished_name.isEmpty())
        emit objectDragFinished(finished_name);
}

void Viewer3D::apply_gizmo_drag_delta(const QPoint& delta_px)
{
    if (!gizmo_drag_active_ || selected_object_name_.isEmpty() || !camera_ || !gizmo_tf_)
        return;

    const float cam_dist = (camera_->position() - camera_->viewCenter()).length();
    const float move_gain = std::max(0.0008f, cam_dist * 0.0014f);   // meters per pixel
    const float rot_gain = 0.006f;                                    // rad per pixel
    const float dpx_x = -static_cast<float>(delta_px.x());
    const float dpx_y = -static_cast<float>(delta_px.y());

    switch (gizmo_mode_)
    {
        case GizmoMode::MoveX:
        {
            // Emit the translation; translate_furniture_object updates
            // furniture_centers_world_, and show_gizmo_for_object (below) snaps
            // the gizmo to the authoritative furniture position.
            const float dx_world = dpx_x * move_gain;
            emit objectTranslateRequested(selected_object_name_, -dx_world, 0.f);
            break;
        }
        case GizmoMode::MoveY:
        {
            const float dy_world = dpx_y * move_gain;
            emit objectTranslateRequested(selected_object_name_, 0.f, dy_world);
            break;
        }
        case GizmoMode::MoveZ:
        {
            // Vertical (height) axis: move the furniture group in world Y.
            const float dz_world = -dpx_y * move_gain;
            const float old_y = gizmo_tf_->translation().y();
            const float new_y = std::max(0.05f, old_y + dz_world);
            const float actual_dy = new_y - old_y;
            if (std::abs(actual_dy) > 1e-6f)
                vertical_move_furniture_object(selected_object_name_, actual_dy);
            break;
        }
        case GizmoMode::RotX:
        {
            const float da = (dpx_x - 0.60f * dpx_y) * rot_gain;
            emit objectRotateRequested(selected_object_name_, da, QVector3D(1.f, 0.f, 0.f));
            break;
        }
        case GizmoMode::RotY:
        {
            const float da = (dpx_x - 0.60f * dpx_y) * rot_gain;
            emit objectRotateRequested(selected_object_name_, da, QVector3D(0.f, 0.f, 1.f));
            break;
        }
        case GizmoMode::RotZ:
        {
            const float da = (dpx_x - 0.60f * dpx_y) * rot_gain;
            emit objectRotateRequested(selected_object_name_, da, QVector3D(0.f, 1.f, 0.f));
            break;
        }
        default:
            break;
    }

    // Always snap the gizmo to the authoritative furniture position so it
    // cannot visually drift away from the object it belongs to.
    show_gizmo_for_object(selected_object_name_);
}

bool Viewer3D::eventFilter(QObject* watched, QEvent* event)
{
    if (watched != container_ && watched != window_)
        return QObject::eventFilter(watched, event);

    if (event->type() == QEvent::MouseMove)
    {
        auto* me = static_cast<QMouseEvent*>(event);
        if (gizmo_drag_active_)
        {
            const QPoint delta = me->pos() - gizmo_last_mouse_pos_;
            gizmo_last_mouse_pos_ = me->pos();
            apply_gizmo_drag_delta(delta);
            return true;
        }
        // Keep camera orbit pivot aligned to the selected gizmo during left-drag
        // (orbit).  Restrict to left button only — right-drag is pan and must not
        // have its viewCenter snapped back, otherwise pan fights the gizmo lock and
        // the camera ends up in erratic positions.
        if (gizmo_root_ && gizmo_root_->isEnabled() && gizmo_tf_ && camera_
            && (me->buttons() & Qt::LeftButton)
            && !(me->buttons() & Qt::RightButton))
        {
            camera_->setViewCenter(gizmo_tf_->translation());
        }
        update_gizmo_screen_scale();
    }
    else if (event->type() == QEvent::Wheel || event->type() == QEvent::Resize)
    {
        if (event->type() == QEvent::Wheel
            && gizmo_root_ && gizmo_root_->isEnabled() && gizmo_tf_ && camera_)
        {
            camera_->setViewCenter(gizmo_tf_->translation());
        }
        update_gizmo_screen_scale();
    }
    else if (event->type() == QEvent::MouseButtonPress)
    {
        // ---- Screen-space proximity picking for gizmo handles ----------------
        // Qt3D TrianglePicking is unreliable for the thin gizmo cylinders / arcs.
        // Instead, project each handle to screen coordinates and check pixel
        // distance from the click.  If a handle is close, start the drag here
        // and consume the event so Qt3D doesn't orbit the camera or pick furniture.
        auto* me = static_cast<QMouseEvent*>(event);
        if (me->button() == Qt::LeftButton
            && gizmo_root_ && gizmo_root_->isEnabled()
            && gizmo_tf_ && camera_ && !selected_object_name_.isEmpty()
            && container_->width() > 0 && container_->height() > 0)
        {
            const QPointF click(me->pos());
            const int vw = container_->width();
            const int vh = container_->height();
            const QMatrix4x4 view = camera_->viewMatrix();
            const QMatrix4x4 proj = camera_->projectionMatrix();
            const QRect vp(0, 0, vw, vh);

            // World → screen (Y-down widget coords).
            auto to_screen = [&](const QVector3D& w) -> QPointF
            {
                const QVector3D s = w.project(view, proj, vp);
                return QPointF(static_cast<double>(s.x()),
                               static_cast<double>(vh) - static_cast<double>(s.y()));
            };

            // Point-to-line-segment distance in 2D.
            auto dist_to_seg = [](QPointF p, QPointF a, QPointF b) -> double
            {
                const double abx = b.x() - a.x(), aby = b.y() - a.y();
                const double len2 = abx * abx + aby * aby;
                if (len2 < 1.0)
                {
                    const double dx = p.x() - a.x(), dy = p.y() - a.y();
                    return std::sqrt(dx * dx + dy * dy);
                }
                double t = ((p.x() - a.x()) * abx + (p.y() - a.y()) * aby) / len2;
                t = std::clamp(t, 0.0, 1.0);
                const double cx = a.x() + t * abx - p.x();
                const double cy = a.y() + t * aby - p.y();
                return std::sqrt(cx * cx + cy * cy);
            };

            const QVector3D gc = gizmo_tf_->translation();
            const float sc = gizmo_tf_->scale();
            const float axis_end_u = gizmo_axis_len_ * 0.69f;  // unscaled
            const float arc_off_u  = axis_end_u * 0.78f;
            const float arc_rad_u  = gizmo_axis_len_ * 0.085f;

            const QPointF pc = to_screen(gc);

            struct Hit { GizmoMode mode; double dist; };
            std::vector<Hit> hits;

            constexpr double move_thr = 18.0;  // pixels
            constexpr double rot_thr  = 22.0;  // pixels

            // ---- Translation axes (line segments from center to tip) ----
            // X red  → world X  → MoveX
            // Z blue → world Y  → MoveZ
            // Y green→ world Z  → MoveY
            {
                double d;
                d = dist_to_seg(click, pc,
                        to_screen(gc + QVector3D(axis_end_u * sc, 0.f, 0.f)));
                if (d < move_thr) hits.push_back({GizmoMode::MoveX, d});

                d = dist_to_seg(click, pc,
                        to_screen(gc + QVector3D(0.f, axis_end_u * sc, 0.f)));
                if (d < move_thr) hits.push_back({GizmoMode::MoveZ, d});

                d = dist_to_seg(click, pc,
                        to_screen(gc + QVector3D(0.f, 0.f, axis_end_u * sc)));
                if (d < move_thr) hits.push_back({GizmoMode::MoveY, d});
            }

            // ---- Rotation arcs (sample points along each 270° arc) ----
            // From init_gizmo:
            //   RotX → euler(0,0,90)  at (arc_off, 0, 0)
            //   RotY → euler(90,0,0)  at (0, 0, arc_off)
            //   RotZ → euler(0,0,0)   at (0, arc_off, 0)
            auto check_arc = [&](const QVector3D& arc_trans_u,
                                 const QVector3D& euler_deg,
                                 GizmoMode mode)
            {
                const QQuaternion arc_rot = QQuaternion::fromEulerAngles(euler_deg);
                constexpr int samples = 24;
                constexpr float start_a = -135.f;
                constexpr float span_a  = 270.f;
                double min_d = 1e6;

                for (int i = 0; i <= samples; ++i)
                {
                    const float a = qDegreesToRadians(
                        start_a + span_a * (static_cast<float>(i) / static_cast<float>(samples)));
                    const QVector3D local(arc_rad_u * std::cos(a), 0.f, arc_rad_u * std::sin(a));
                    const QVector3D world = gc + sc * (arc_trans_u + arc_rot.rotatedVector(local));
                    const QPointF sp = to_screen(world);
                    const double dx = click.x() - sp.x();
                    const double dy = click.y() - sp.y();
                    min_d = std::min(min_d, std::sqrt(dx * dx + dy * dy));
                }
                if (min_d < rot_thr) hits.push_back({mode, min_d});
            };

            check_arc(QVector3D(arc_off_u, 0.f, 0.f),  QVector3D(0.f, 0.f, 90.f), GizmoMode::RotX);
            check_arc(QVector3D(0.f, 0.f, arc_off_u),   QVector3D(90.f, 0.f, 0.f), GizmoMode::RotY);
            check_arc(QVector3D(0.f, arc_off_u, 0.f),   QVector3D(0.f, 0.f, 0.f),  GizmoMode::RotZ);

            if (!hits.empty())
            {
                auto best = std::min_element(hits.begin(), hits.end(),
                    [](const Hit& a, const Hit& b) { return a.dist < b.dist; });
                gizmo_last_mouse_pos_ = me->pos();
                begin_gizmo_drag(best->mode);
                return true;  // consume event: no camera orbit, no Qt3D pick
            }
        }
    }
    else if (event->type() == QEvent::MouseButtonRelease)
    {
        auto* me = static_cast<QMouseEvent*>(event);
        if (me->button() == Qt::LeftButton && gizmo_drag_active_)
        {
            end_gizmo_drag();
            return true;
        }
    }

    return QObject::eventFilter(watched, event);
}


// ---------------------------------------------------------------------------
// attach_picker – add a QObjectPicker to entity e; on Shift+Right-click
//                 show an overlay label with the object name for 3 s.
// ---------------------------------------------------------------------------
void Viewer3D::attach_picker(Qt3DCore::QEntity* e, const QString& name)
{
    auto* picker = new Qt3DRender::QObjectPicker(e);
    picker->setHoverEnabled(false);
    picker->setDragEnabled(false);
    if (name == "__GIZMO_MOVE_Z")
        // Highest priority for vertical handle: it is visually close to ROT_Z arc.
        picker->setPriority(280);
    else if (name.startsWith("__GIZMO_ROT_"))
        // Rotation arcs should win over horizontal move handles.
        picker->setPriority(260);
    else if (name.startsWith("__GIZMO_MOVE_"))
        picker->setPriority(230);
    else if (name == "Floor")
        picker->setPriority(-20);
    else
        picker->setPriority(10);
    e->addComponent(picker);

    connect(picker, &Qt3DRender::QObjectPicker::pressed,
            this, [this, name](Qt3DRender::QPickEvent* ev)
    {
        auto gizmo_mode_from_name = [&](const QString& n) -> GizmoMode
        {
            if (n == "__GIZMO_MOVE_X") return GizmoMode::MoveX;
            if (n == "__GIZMO_MOVE_Y") return GizmoMode::MoveY;
            if (n == "__GIZMO_MOVE_Z") return GizmoMode::MoveZ;
            if (n == "__GIZMO_ROT_X")  return GizmoMode::RotX;
            if (n == "__GIZMO_ROT_Y")  return GizmoMode::RotY;
            if (n == "__GIZMO_ROT_Z")  return GizmoMode::RotZ;
            return GizmoMode::None;
        };

        auto resolve_gizmo_mode = [&](const QString& picked_name,
                                      const QVector3D& hit_world) -> GizmoMode
        {
            GizmoMode mode = gizmo_mode_from_name(picked_name);
            if (!picked_name.startsWith("__GIZMO_MOVE_") || !gizmo_tf_)
                return mode;

            // Resolve translation handle from hit direction to avoid ambiguous picks
            // between overlapping move handles when viewed at steep camera angles.
            const QVector3D d = hit_world - gizmo_tf_->translation();
            const float ax = std::abs(d.x());
            const float ay = std::abs(d.y());
            const float az = std::abs(d.z());

            if (ay >= ax && ay >= az) return GizmoMode::MoveZ;
            if (az >= ax && az >= ay) return GizmoMode::MoveY;
            return GizmoMode::MoveX;
        };

        const GizmoMode gizmo_mode = resolve_gizmo_mode(name, ev->worldIntersection());
        // Allow both plain Left and Shift+Left on gizmo handles to start drag.
        if (gizmo_mode != GizmoMode::None
            && ev->button() == Qt3DRender::QPickEvent::LeftButton
            && gizmo_root_ && gizmo_root_->isEnabled())
        {
            gizmo_last_mouse_pos_ = QPoint(static_cast<int>(ev->position().x()),
                                           static_cast<int>(ev->position().y()));
            begin_gizmo_drag(gizmo_mode);
            return;
        }

        if (ev->button() == Qt3DRender::QPickEvent::LeftButton)
        {
            // Ignore plain left clicks on gizmo parts so camera orbit does not hide it.
            if (name.startsWith("__GIZMO_"))
                return;

            if (name != "Floor" && !name.startsWith("Wall ") && name != "Robot")
            {
                show_gizmo_for_object(name);
                emit objectLeftClicked(name);
            }
            // Floor / Wall / Robot clicks are ignored for selection purposes;
            // the gizmo stays on the previously selected furniture.

            // Keep plain left-drag orbit stable (no implicit translation on press).
            // Optional explicit recenter: Alt+Left click.
            if (camera_ && (ev->modifiers() & static_cast<int>(Qt::AltModifier)))
                camera_->setViewCenter(ev->worldIntersection());
            return;
        }

        if (ev->button() == Qt3DRender::QPickEvent::RightButton)
        {
            if (ev->modifiers() & static_cast<int>(Qt::ShiftModifier))
            {
                qInfo() << "[Viewer3D] Picked:" << name;
                if (name == "Floor") {
                    float room_x = -ev->worldIntersection().x();
                    float room_y = ev->worldIntersection().z();
                    emit floorPicked(room_x, room_y);
                } else {
                    emit objectPicked(name);
                }

                if (pick_label_ && container_)
                {
                    pick_label_->setText(name);
                    pick_label_->adjustSize();
                    // Centre at the bottom of the 3D viewport
                    pick_label_->move(
                        (container_->width()  - pick_label_->width())  / 2,
                         container_->height() - pick_label_->height()  - 16);
                    pick_label_->show();
                    pick_label_->raise();
                    pick_timer_->start(3000);
                }
            }
            else if (ev->modifiers() & static_cast<int>(Qt::ControlModifier))
            {
                qInfo() << "[Viewer3D] Mission cancel requested via object:" << name;
                emit cancelMissionRequested();

                if (pick_label_ && container_)
                {
                    pick_label_->setText("CANCELLED");
                    pick_label_->adjustSize();
                    pick_label_->move(
                        (container_->width()  - pick_label_->width())  / 2,
                         container_->height() - pick_label_->height()  - 16);
                    pick_label_->show();
                    pick_label_->raise();
                    pick_timer_->start(3000);
                }
            }
        }
    });
}

// ---------------------------------------------------------------------------
// init_lidar_pool  – pre-create shared sphere meshes + per-point transforms
// ---------------------------------------------------------------------------
void Viewer3D::init_lidar_pool()
{
    static const QVector3D hidden(0.f, -500.f, 0.f);

    auto make_pool = [&](int count, float radius,
                         const QColor& diffuse, const QColor& ambient,
                         std::vector<Qt3DCore::QTransform*>& xf_out)
    {
        // One shared mesh and material for the whole pool layer
        auto* mesh = new Qt3DExtras::QSphereMesh(root_entity_);
        mesh->setRadius(radius);
        mesh->setRings(4);
        mesh->setSlices(6);

        auto* mat = new Qt3DExtras::QPhongMaterial(root_entity_);
        mat->setDiffuse(diffuse);
        mat->setAmbient(ambient);
        mat->setShininess(0.f);
        mat->setSpecular(QColor(0,0,0));

        xf_out.reserve(count);
        for (int i = 0; i < count; ++i)
        {
            auto* e  = new Qt3DCore::QEntity(root_entity_);
            auto* tf = new Qt3DCore::QTransform(e);
            tf->setTranslation(hidden);
            e->addComponent(mesh);
            e->addComponent(mat);
            e->addComponent(tf);
            xf_out.push_back(tf);
        }
    };

    make_pool(kLidarPoolHigh, 0.04f, QColor(50, 230, 50),  QColor(20, 100, 20),  lidar_hi_xf_);
    make_pool(kLidarPoolLow,  0.04f, QColor(50, 220, 220), QColor(20,  90, 90),  lidar_lo_xf_);
    make_pool(kSegmentedPool, 0.03f, QColor(70, 140, 255), QColor(25, 55, 120), segmented_xf_);
}

// ---------------------------------------------------------------------------
// update_lidar_points  – reposition pre-allocated pool spheres each frame
// ---------------------------------------------------------------------------
void Viewer3D::update_lidar_points(
    const std::vector<Eigen::Vector3f>& pts_hi,
    const std::vector<Eigen::Vector3f>& pts_lo,
    float robot_x, float robot_y, float robot_theta_rad)
{
    static const QVector3D hidden(0.f, -500.f, 0.f);

    const float cos_t = std::cos(robot_theta_rad);
    const float sin_t = std::sin(robot_theta_rad);

    // Transform robot-frame 3D point → Qt3D world position (with X-flip)
    auto to_world = [&](const Eigen::Vector3f& p) -> QVector3D
    {
        const float wx =  cos_t * p.x() - sin_t * p.y() + robot_x;   // room X
        const float wz =  sin_t * p.x() + cos_t * p.y() + robot_y;   // room Y → Qt3D Z
        return QVector3D(-wx, p.z(), wz);   // negate X for scene consistency
    };

    auto fill = [&](const std::vector<Eigen::Vector3f>& pts,
                    std::vector<Qt3DCore::QTransform*>& pool)
    {
        const int pool_size = static_cast<int>(pool.size());
        const int pt_count  = static_cast<int>(pts.size());
        const int stride    = (pt_count > pool_size) ? (pt_count / pool_size) : 1;
        int idx = 0;
        for (int i = 0; i < pt_count && idx < pool_size; i += stride, ++idx)
            pool[idx]->setTranslation(to_world(pts[i]));
        for (int i = idx; i < pool_size; ++i)
            pool[i]->setTranslation(hidden);
    };

    fill(pts_hi, lidar_hi_xf_);
    fill(pts_lo, lidar_lo_xf_);
}

// ---------------------------------------------------------------------------
// update_segmented_points – room-frame points shown as red cloud
// ---------------------------------------------------------------------------
void Viewer3D::update_segmented_points(const std::vector<Eigen::Vector3f>& points_layout)
{
    static const QVector3D hidden(0.f, -500.f, 0.f);

    const int pool_size = static_cast<int>(segmented_xf_.size());
    const int pt_count  = static_cast<int>(points_layout.size());
    const int stride    = (pt_count > pool_size && pool_size > 0) ? (pt_count / pool_size) : 1;

    int idx = 0;
    for (int i = 0; i < pt_count && idx < pool_size; i += stride, ++idx)
    {
        const auto &p = points_layout[i];
        segmented_xf_[idx]->setTranslation(QVector3D(-p.x(), p.z(), p.y()));
    }
    for (int i = idx; i < pool_size; ++i)
        segmented_xf_[i]->setTranslation(hidden);
}

// ---------------------------------------------------------------------------
// update_segmented_boxes – oriented 3D boxes + label per segmented object
// ---------------------------------------------------------------------------
void Viewer3D::update_segmented_boxes(const std::vector<SegmentedBoxItem>& boxes)
{
    for (auto* e : segmented_box_entities_)
    {
        e->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
        delete e;
    }
    segmented_box_entities_.clear();

    static const QColor edge_diffuse(90, 170, 255);
    static const QColor edge_ambient(30, 75, 140);
    constexpr float edge_thickness = 0.015f;

    for (const auto& box : boxes)
    {
        const float sx = std::max(0.05f, box.size.x());
        const float sy = std::max(0.05f, box.size.y());
        const float sz = std::max(0.05f, box.size.z());

        auto* box_entity = new Qt3DCore::QEntity(root_entity_);
        auto* box_tf = new Qt3DCore::QTransform(box_entity);
        box_tf->setTranslation(QVector3D(-box.center.x(), box.center_height, box.center.y()));
        box_tf->setRotationY(qRadiansToDegrees(box.yaw_rad));
        box_entity->addComponent(box_tf);

        auto make_edge = [&](const QVector3D& local_center,
                             float ex, float ey, float ez,
                             bool add_picker)
        {
            auto* edge_e = new Qt3DCore::QEntity(box_entity);
            auto* edge_mesh = new Qt3DExtras::QCuboidMesh();
            edge_mesh->setXExtent(ex);
            edge_mesh->setYExtent(ey);
            edge_mesh->setZExtent(ez);

            auto* edge_mat = new Qt3DExtras::QPhongMaterial(edge_e);
            edge_mat->setDiffuse(edge_diffuse);
            edge_mat->setAmbient(edge_ambient);
            edge_mat->setSpecular(QColor(180, 220, 255));
            edge_mat->setShininess(30.f);

            auto* edge_tf = new Qt3DCore::QTransform(edge_e);
            edge_tf->setTranslation(local_center);

            edge_e->addComponent(edge_mesh);
            edge_e->addComponent(edge_mat);
            edge_e->addComponent(edge_tf);
            if (add_picker)
                attach_picker(edge_e, QString::fromStdString(box.label));
        };

        const float hx = sx * 0.5f;
        const float hy = sy * 0.5f;
        const float hz = sz * 0.5f;

        // 4 edges parallel to X
        make_edge(QVector3D(0.f, -hy, -hz), sx, edge_thickness, edge_thickness, true);
        make_edge(QVector3D(0.f, -hy,  hz), sx, edge_thickness, edge_thickness, false);
        make_edge(QVector3D(0.f,  hy, -hz), sx, edge_thickness, edge_thickness, false);
        make_edge(QVector3D(0.f,  hy,  hz), sx, edge_thickness, edge_thickness, false);
        // 4 edges parallel to Y
        make_edge(QVector3D(-hx, 0.f, -hz), edge_thickness, sy, edge_thickness, false);
        make_edge(QVector3D(-hx, 0.f,  hz), edge_thickness, sy, edge_thickness, false);
        make_edge(QVector3D( hx, 0.f, -hz), edge_thickness, sy, edge_thickness, false);
        make_edge(QVector3D( hx, 0.f,  hz), edge_thickness, sy, edge_thickness, false);
        // 4 edges parallel to Z
        make_edge(QVector3D(-hx, -hy, 0.f), edge_thickness, edge_thickness, sz, false);
        make_edge(QVector3D(-hx,  hy, 0.f), edge_thickness, edge_thickness, sz, false);
        make_edge(QVector3D( hx, -hy, 0.f), edge_thickness, edge_thickness, sz, false);
        make_edge(QVector3D( hx,  hy, 0.f), edge_thickness, edge_thickness, sz, false);

        segmented_box_entities_.push_back(box_entity);

        auto* text_entity = new Qt3DCore::QEntity(root_entity_);
        auto* text_mesh = new Qt3DExtras::QExtrudedTextMesh();
        QFont f("Sans Serif", 24, QFont::Bold);
        text_mesh->setFont(f);
        text_mesh->setText(QString::fromStdString(box.label));
        text_mesh->setDepth(0.02f);

        auto* text_mat = new Qt3DExtras::QPhongMaterial(text_entity);
        text_mat->setDiffuse(QColor(245, 245, 245));
        text_mat->setAmbient(QColor(180, 180, 180));
        text_mat->setSpecular(QColor(0, 0, 0));
        text_mat->setShininess(0.f);

        auto* text_tf = new Qt3DCore::QTransform(text_entity);
        text_tf->setTranslation(QVector3D(-box.center.x(), box.center_height + 0.02f, box.center.y()));
        text_tf->setRotationY(qRadiansToDegrees(box.yaw_rad));
        text_tf->setScale(0.015f);

        text_entity->addComponent(text_mesh);
        text_entity->addComponent(text_mat);
        text_entity->addComponent(text_tf);
        segmented_box_entities_.push_back(text_entity);
    }
}

// ---------------------------------------------------------------------------
// Path visualization (Ribbon geometry)
// ---------------------------------------------------------------------------
void Viewer3D::init_path_entity()
{
    path_entity_ = new Qt3DCore::QEntity(root_entity_);

    auto* material = new Qt3DExtras::QPhongMaterial(path_entity_);
    material->setAmbient(QColor(0, 210, 60));   // Green path
    material->setDiffuse(QColor(0, 255, 80));
    material->setShininess(0.0f);

    path_geo_ = new Qt3DCore::QGeometry(path_entity_);
    path_buf_ = new Qt3DCore::QBuffer(path_geo_);
    path_attr_ = new Qt3DCore::QAttribute(path_geo_);
    path_attr_->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
    path_attr_->setBuffer(path_buf_);
    path_attr_->setVertexBaseType(Qt3DCore::QAttribute::Float);
    path_attr_->setVertexSize(3);
    path_attr_->setByteOffset(0);
    path_attr_->setByteStride(3 * sizeof(float));
    path_attr_->setName(Qt3DCore::QAttribute::defaultPositionAttributeName());
    path_geo_->addAttribute(path_attr_);

    path_renderer_ = new Qt3DRender::QGeometryRenderer(path_entity_);
    path_renderer_->setGeometry(path_geo_);
    // TriangleStrip ribbon: two vertices per waypoint give world-space width
    // that stays constant regardless of camera zoom.
    path_renderer_->setPrimitiveType(Qt3DRender::QGeometryRenderer::TriangleStrip);

    path_entity_->addComponent(path_renderer_);
    path_entity_->addComponent(material);
}

void Viewer3D::update_path(const std::vector<Eigen::Vector2f>& path)
{
    if (!path_entity_) init_path_entity();

    if (path.size() < 2) {
        path_renderer_->setVertexCount(0);
        return;
    }

    // Build a triangle-strip ribbon with world-space half-width so the path
    // remains clearly visible at any zoom level.
    constexpr float HALF_W = 0.04f;  // 4 cm half-width (8 cm total ribbon)
    constexpr float H      = 0.04f;  // 4 cm above floor to avoid Z-fighting

    // Two vertices per waypoint → 2*N vertices, 3 floats each.
    const std::size_t N = path.size();
    QByteArray data;
    data.resize(static_cast<int>(N * 2 * 3 * sizeof(float)));
    auto* raw = reinterpret_cast<float*>(data.data());

    for (std::size_t i = 0; i < N; ++i)
    {
        // Compute a perpendicular offset in the XZ plane.
        Eigen::Vector2f dir;
        if (i == 0)
            dir = (path[1] - path[0]);
        else if (i == N - 1)
            dir = (path[N - 1] - path[N - 2]);
        else
            dir = (path[i + 1] - path[i - 1]);

        float len = dir.norm();
        if (len < 1e-6f) dir = Eigen::Vector2f(1.f, 0.f);
        else             dir /= len;

        // Perpendicular in 2D: rotate 90°
        Eigen::Vector2f perp(-dir.y(), dir.x());

        Eigen::Vector2f left  = path[i] + HALF_W * perp;
        Eigen::Vector2f right = path[i] - HALF_W * perp;

        // Left vertex  (room X → scene -X, room Y → scene +Z)
        raw[i * 6 + 0] = -left.x();
        raw[i * 6 + 1] =  H;
        raw[i * 6 + 2] =  left.y();
        // Right vertex
        raw[i * 6 + 3] = -right.x();
        raw[i * 6 + 4] =  H;
        raw[i * 6 + 5] =  right.y();
    }

    path_buf_->setData(data);
    path_attr_->setCount(static_cast<uint>(N * 2));
    path_renderer_->setVertexCount(static_cast<int>(N * 2));
}

} // namespace rc

