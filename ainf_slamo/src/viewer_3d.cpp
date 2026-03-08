#ifdef HAS_QT3D

#include "viewer_3d.h"

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QDirectionalLight>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DRender/QMesh>
#include <QSizePolicy>
#include <QCoreApplication>
#include <QDir>
#include <cmath>
#include <algorithm>

namespace rc {

// ---------------------------------------------------------------------------
// WebotsStyleCameraController::moveCamera
// Webots 3D viewport:
//   Left drag   → orbit (rotate scene around view centre)
//   Wheel       → zoom (dolly)
//   Right drag  → pan (translate camera + view centre)
// ---------------------------------------------------------------------------
void WebotsStyleCameraController::moveCamera(
    const Qt3DExtras::QAbstractCameraController::InputState& state, float dt)
{
    auto* cam = camera();
    if (!cam) return;

    // NOTE: Qt3D always delivers mouse-move deltas in rx/ryAxisValue.
    //       tx/tyAxisValue are keyboard axes only – do NOT use them for mouse pan.

    // ---- Orbit: left button drag ------------------------------------------
    if (state.leftMouseButtonActive && !state.rightMouseButtonActive)
    {
        cam->panAboutViewCenter ( state.rxAxisValue * lookSpeed() * dt);
        cam->tiltAboutViewCenter(-state.ryAxisValue * lookSpeed() * dt);
    }

    // ---- Pan: right button drag (same rx/ry axes, different camera op) ----
    if (state.rightMouseButtonActive)
    {
        // Translate in camera-local XY (screen plane), moving view centre too
        cam->translate(QVector3D(-state.rxAxisValue * linearSpeed() * dt,
                                  state.ryAxisValue * linearSpeed() * dt,
                                  0.f),
                       Qt3DRender::QCamera::TranslateViewCenter);
    }

    // ---- Zoom: scroll wheel -----------------------------------------------
    if (qAbs(state.tzAxisValue) > 0.f)
    {
        cam->translate(QVector3D(0.f, 0.f, -state.tzAxisValue * linearSpeed() * dt),
                       Qt3DRender::QCamera::DontTranslateViewCenter);
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

    container_ = QWidget::createWindowContainer(window_, parent);
    container_->setMinimumSize(350, 250);
    container_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container_->setFocusPolicy(Qt::StrongFocus);   // allow keyboard input for camera

    // ---- Root entity -------------------------------------------------------
    root_entity_ = new Qt3DCore::QEntity();

    // ---- Camera ------------------------------------------------------------
    Qt3DRender::QCamera* camera = window_->camera();
    camera->lens()->setPerspectiveProjection(45.f, 16.f / 9.f, 0.05f, 500.f);
    camera->setPosition(QVector3D(0.f, 15.f, 15.f));
    camera->setViewCenter(QVector3D(0.f, 0.f, 0.f));
    camera->setUpVector(QVector3D(0.f, 1.f, 0.f));

    auto* camCtrl = new WebotsStyleCameraController(root_entity_);
    camCtrl->setCamera(camera);
    camCtrl->setLinearSpeed(20.f);
    camCtrl->setLookSpeed(180.f);

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
    //   Step 2: 180° around Y  → faces +X in room frame (fixes "looking backwards")
    robot_base_rot_ = QQuaternion::fromAxisAndAngle(0.f, 1.f, 0.f, 180.f)
                    * QQuaternion::fromAxisAndAngle(1.f, 0.f, 0.f, -90.f);
    robot_transform_->setTranslation(QVector3D(0.f, robot_half_h_, 0.f));  // X=0 at start, negated when updated

    robotEntity->addComponent(robotMesh);
    robotEntity->addComponent(robotMat);
    robotEntity->addComponent(robot_transform_);

    // ---- Pre-allocate lidar point pool ------------------------------------
    init_lidar_pool();

    // ---- Commit root entity ------------------------------------------------
    window_->setRootEntity(root_entity_);
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
    // In Qt3D (Y-up) that maps to a rotation around +Y by -theta_rad.
    const QQuaternion heading = QQuaternion::fromAxisAndAngle(0.f, 1.f, 0.f,
                                    qRadiansToDegrees(-theta_rad));
    // Apply base correction first, then heading on top.
    robot_transform_->setRotation(heading * robot_base_rot_);
}

// ---------------------------------------------------------------------------
// update_obstacles  – called after temp_obstacles_ list changes
// ---------------------------------------------------------------------------
void Viewer3D::update_obstacles(const std::vector<std::vector<Eigen::Vector2f>>& obs_polys)
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

    for (const auto& poly : obs_polys)
    {
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
        const float h     = wall_height_ * 0.85f;  // slightly shorter than walls

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

    // Base correction: Z-up meshes → Y-up.  -90° around X.
    const QQuaternion base_rot = QQuaternion::fromAxisAndAngle(1.f, 0.f, 0.f, -90.f);

    const QString component_root = QDir(QCoreApplication::applicationDirPath()
                                        + "/..").absolutePath();
    auto abs_mesh = [&](const QString& rel) -> QString {
        return component_root + "/" + rel;
    };

    // Helper: create and register one entity
    auto make_entity = [&](const QString&       mesh_file,
                           const QColor&        diffuse,
                           const QColor&        ambient,
                           float                shininess,
                           const Eigen::Vector2f& centroid,
                           const QQuaternion&   rot,
                           const QVector3D&     scale3d) -> void
    {
        auto* e = new Qt3DCore::QEntity(root_entity_);

        auto* mesh = new Qt3DRender::QMesh(e);
        mesh->setSource(QUrl::fromLocalFile(mesh_file));

        auto* mat = new Qt3DExtras::QPhongMaterial(e);
        mat->setDiffuse(diffuse);
        mat->setAmbient(ambient);
        mat->setShininess(shininess);

        auto* tf = new Qt3DCore::QTransform(e);
        tf->setTranslation(QVector3D(-centroid.x(), 0.f, centroid.y()));
        tf->setRotation(rot);
        tf->setScale3D(scale3d);

        e->addComponent(mesh);
        e->addComponent(mat);
        e->addComponent(tf);

        furniture_entities_.push_back(e);
    };

    for (const auto& item : items)
    {
        const QString ql = QString::fromStdString(item.label).toLower();
        const Eigen::Vector2f& cen  = item.centroid;
        const Eigen::Vector2f& size = item.size;

        if (ql.contains("mesa") || ql.contains("table"))
        {
            // Scale table mesh (native 1.2 m × 0.8 m in obj X × Y) to SVG AABB.
            // After -90°X rotation: obj-X→room-X, obj-Y→room-Z  → scale accordingly.
            const float sx = (size.x() > 0.1f) ? size.x() / 1.2f : 1.f;
            const float sy = (size.y() > 0.1f) ? size.y() / 0.8f : 1.f;
            make_entity(abs_mesh("meshes/table.obj"),
                        QColor(160, 100, 55), QColor(70, 44, 24), 20.f,
                        cen, base_rot, QVector3D(sx, sy, 1.f));
        }
        else if (ql.contains("banco") || ql.contains("bench"))
        {
            const QQuaternion bench_rot =
                QQuaternion::fromAxisAndAngle(0.f, 1.f, 0.f, 90.f) * base_rot;
            make_entity(abs_mesh("meshes/bench.obj"),
                        QColor(130, 85, 45), QColor(55, 36, 19), 15.f,
                        cen, bench_rot, QVector3D(1.f, 1.f, 1.f));
        }
        else if (ql.contains("maceta") || ql.contains("plant") || ql.contains("pot"))
        {
            // Two entities at same position: light-brown pot + green tree
            make_entity(abs_mesh("meshes/pot_only.obj"),
                        QColor(195, 155, 95), QColor(90, 70, 40), 10.f,
                        cen, base_rot, QVector3D(1.f, 1.f, 1.f));
            make_entity(abs_mesh("meshes/tree_only.obj"),
                        QColor(55, 130, 45), QColor(25, 60, 20), 5.f,
                        cen, base_rot, QVector3D(1.f, 1.f, 1.f));
        }
        else if (ql.contains("vitrina") || ql.contains("cabinet") || ql.contains("shelf"))
        {
            make_entity(abs_mesh("meshes/bench.obj"),
                        QColor(180, 200, 220), QColor(70, 80, 90), 40.f,
                        cen, base_rot, QVector3D(1.f, 1.f, 1.f));
        }
        else
        {
            qWarning() << "[Viewer3D] No mesh mapping for furniture label:"
                       << QString::fromStdString(item.label);
            continue;
        }

        qInfo() << "[Viewer3D] Furniture:" << QString::fromStdString(item.label)
                << "at (" << cen.x() << "," << cen.y() << ")"
                << "size (" << size.x() << "x" << size.y() << ")";
    }
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

} // namespace rc

#endif // HAS_QT3D
