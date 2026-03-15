#include "camera_viewer.h"

#include <QImage>
#include <QPixmap>
#include <QDateTime>
#include <QSizePolicy>
#include <QPainter>
#include <QPen>
#include <QPolygon>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <print>

namespace
{
enum class InfraMaskMode
{
    WallsOnly,
    AllPlanes
};

InfraMaskMode read_infra_mask_mode()
{
    const char* s = std::getenv("AINFO_INFRA_MASK_MODE");
    if (s == nullptr)
        return InfraMaskMode::WallsOnly;

    std::string mode(s);
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode == "all" || mode == "allplanes" || mode == "wall_floor_ceiling")
        return InfraMaskMode::AllPlanes;
    return InfraMaskMode::WallsOnly;
}

bool extract_depth_buffer_meters(const RoboCompImageSegmentation::TDepth& depth,
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

    // uint16 and uint8 formats are usually delivered in millimeters.
    if (nbytes < expected * sizeof(float))
    {
        for (float& d : out)
            d = std::isfinite(d) ? d * 0.001f * scale : 0.f;
    }

    return true;
}

bool extract_depth_from_tdata(const RoboCompImageSegmentation::TData& tdata,
                              int& width,
                              int& height,
                              std::vector<float>& out)
{
    return extract_depth_buffer_meters(tdata.depth, width, height, out);
}

QColor depth_to_color(float d, float dmin, float dmax)
{
    if (!std::isfinite(d) || d <= 0.f || dmax <= dmin)
        return QColor(0, 0, 0);
    const float t = std::clamp((d - dmin) / (dmax - dmin), 0.f, 1.f);
    const int hue = static_cast<int>((1.f - t) * 240.f);
    return QColor::fromHsv(hue, 255, 255);
}

QImage depth_colormap_image(const std::vector<float>& depth_m, int w, int h)
{
    QImage img(w, h, QImage::Format_RGB888);
    img.fill(Qt::black);
    if (depth_m.empty() || w <= 0 || h <= 0)
        return img;

    float dmin = std::numeric_limits<float>::max();
    float dmax = 0.f;
    for (const float d : depth_m)
    {
        if (!std::isfinite(d) || d <= 0.05f)
            continue;
        dmin = std::min(dmin, d);
        dmax = std::max(dmax, d);
    }
    if (!std::isfinite(dmin) || dmax <= dmin)
        return img;

    for (int y = 0; y < h; ++y)
    {
        uchar* row = img.scanLine(y);
        for (int x = 0; x < w; ++x)
        {
            const float d = depth_m[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)];
            const QColor c = depth_to_color(d, dmin, dmax);
            row[3 * x + 0] = static_cast<uchar>(c.red());
            row[3 * x + 1] = static_cast<uchar>(c.green());
            row[3 * x + 2] = static_cast<uchar>(c.blue());
        }
    }
    return img;
}
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
CameraViewer::CameraViewer(RoboCompImageSegmentation::ImageSegmentationPrxPtr proxy, QWidget* parent)
        : QDialog(parent, Qt::Window), proxy_(std::move(proxy))
{
    setWindowTitle("Camera Viewer");
    setMinimumSize(640, 520);
    resize(600, 700);

    // ---- Tab widget (RGB | Depth) -------------------------------------------
    tabs_ = new QTabWidget(this);

    rgb_label_   = new QLabel(this);
    depth_label_ = new QLabel(this);
    for (auto* lbl : {rgb_label_, depth_label_})
    {
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setMinimumSize(320, 240);
        lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        lbl->setStyleSheet("background: #1a1a2e;");
        lbl->setText("No frame yet");
        lbl->setStyleSheet("background:#1a1a2e; color:#555; font-size:14px;");
    }
    tabs_->addTab(rgb_label_,   "RGB");
    tabs_->addTab(depth_label_, "Depth");

    // ---- Control bar --------------------------------------------------------
    auto* ctrl_bar   = new QWidget(this);
    auto* ctrl_layout = new QHBoxLayout(ctrl_bar);
    ctrl_layout->setContentsMargins(4, 2, 4, 2);

    ctrl_layout->addWidget(new QLabel("FPS:", ctrl_bar));
    fps_spin_ = new QSpinBox(ctrl_bar);
    fps_spin_->setRange(1, 30);
    fps_spin_->setValue(10);
    fps_spin_->setToolTip("Capture frame rate");
    ctrl_layout->addWidget(fps_spin_);

    em_button_ = new QPushButton("EM", ctrl_bar);
    em_button_->setToolTip("Start EM reassignment on visible model objects");
    ctrl_layout->addWidget(em_button_);

    em_accept_button_ = new QPushButton("Accept Fit", ctrl_bar);
    em_accept_button_->setToolTip("Accept the fitted geometry and update the model");
    em_accept_button_->setEnabled(false);
    ctrl_layout->addWidget(em_accept_button_);

    em_reject_button_ = new QPushButton("Reject Fit", ctrl_bar);
    em_reject_button_->setToolTip("Reject the fitted geometry and keep current model");
    em_reject_button_->setEnabled(false);
    ctrl_layout->addWidget(em_reject_button_);

    objects_button_ = new QPushButton("Objects", ctrl_bar);
    objects_button_->setCheckable(true);
    objects_button_->setChecked(false);
    objects_button_->setToolTip("Toggle yellow furniture wireframes on the overlay");
    ctrl_layout->addWidget(objects_button_);
    connect(objects_button_, &QPushButton::toggled, this, [this](bool on){ show_objects_ = on; });

    mask_button_ = new QPushButton("Mask", ctrl_bar);
    mask_button_->setCheckable(true);
    mask_button_->setChecked(false);
    mask_button_->setToolTip("Black out wall/floor/ceiling pixels (infrastructure masking)");
    ctrl_layout->addWidget(mask_button_);
    connect(mask_button_, &QPushButton::toggled, this, [this](bool on){ mask_infrastructure_ = on; });

    auto* raw_button = new QPushButton("Raw Image", ctrl_bar);
    raw_button->setCheckable(true);
    raw_button->setToolTip("Show raw camera image without infrastructure masking or overlays");
    ctrl_layout->addWidget(raw_button);
    connect(raw_button, &QPushButton::toggled, this, [this](bool on){ raw_mode_ = on; });

    ctrl_layout->addStretch();

    status_label_ = new QLabel("Idle", ctrl_bar);
    status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ctrl_layout->addWidget(status_label_);

    // ---- Main layout --------------------------------------------------------
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(4, 4, 4, 4);
    main_layout->addWidget(tabs_, 1);
    main_layout->addWidget(ctrl_bar, 0);

    // ---- Timer --------------------------------------------------------------
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CameraViewer::grab_frame);

    // Update timer period when fps changes
    connect(fps_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int fps) {
        if (timer_->isActive()) timer_->setInterval(1000 / std::max(1, fps));
    });
    connect(em_button_, &QPushButton::clicked, this, &CameraViewer::emRequested);
    connect(em_accept_button_, &QPushButton::clicked, this, &CameraViewer::emAcceptRequested);
    connect(em_reject_button_, &QPushButton::clicked, this, &CameraViewer::emRejectRequested);
}

void CameraViewer::set_period_ms(int ms)
{
    fps_spin_->setValue(std::clamp(1000 / std::max(1, ms), 1, 30));
    timer_->setInterval(ms);
}

void CameraViewer::set_wireframe_segments_camera(
    const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& segments,
    const QString& label)
{
    std::lock_guard<std::mutex> lock(wireframe_mutex_);
    wireframe_segments_camera_ = segments;
    wireframe_segment_colors_camera_.clear();
    wireframe_label_ = label;
}

void CameraViewer::set_wireframe_segments_camera_colored(
    const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& segments,
    const std::vector<QColor>& segment_colors,
    const QString& label)
{
    std::lock_guard<std::mutex> lock(wireframe_mutex_);
    wireframe_segments_camera_ = segments;
    wireframe_segment_colors_camera_ = segment_colors;
    wireframe_label_ = label;
}

void CameraViewer::set_wireframe_annotations_camera(const std::vector<Eigen::Vector3f>& anchor_points_camera,
                                                    const std::vector<QString>& annotation_texts,
                                                    const std::vector<QColor>& annotation_colors)
{
    std::lock_guard<std::mutex> lock(wireframe_mutex_);
    wireframe_annotation_points_camera_ = anchor_points_camera;
    wireframe_annotation_texts_ = annotation_texts;
    wireframe_annotation_colors_ = annotation_colors;
}

void CameraViewer::clear_wireframe_overlay()
{
    std::lock_guard<std::mutex> lock(wireframe_mutex_);
    wireframe_segments_camera_.clear();
    wireframe_segment_colors_camera_.clear();
    wireframe_label_.clear();
    wireframe_annotation_points_camera_.clear();
    wireframe_annotation_texts_.clear();
    wireframe_annotation_colors_.clear();
}

void CameraViewer::set_em_points_overlay(const std::vector<Eigen::Vector3f>& points_camera,
                                         const std::vector<int>& point_classes,
                                         const std::vector<QColor>& class_colors,
                                         const std::vector<QString>& class_labels,
                                         const QString& title)
{
    std::lock_guard<std::mutex> lock(wireframe_mutex_);
    em_points_camera_ = points_camera;
    em_point_classes_ = point_classes;
    em_class_colors_ = class_colors;
    em_class_labels_ = class_labels;
    em_overlay_title_ = title;
}

void CameraViewer::clear_em_points_overlay()
{
    std::lock_guard<std::mutex> lock(wireframe_mutex_);
    em_points_camera_.clear();
    em_point_classes_.clear();
    em_class_colors_.clear();
    em_class_labels_.clear();
    em_overlay_title_.clear();
}

void CameraViewer::set_infrastructure_context(const Eigen::Affine2f& robot_pose,
                                              const std::vector<Eigen::Vector2f>& room_polygon,
                                              float camera_tx,
                                              float camera_ty,
                                              float camera_tz,
                                              float wall_height)
{
    std::lock_guard<std::mutex> lock(wireframe_mutex_);
    infrastructure_ctx_valid_ = !room_polygon.empty();
    infrastructure_robot_pose_ = robot_pose;
    infrastructure_room_polygon_ = room_polygon;
    infrastructure_camera_tx_ = camera_tx;
    infrastructure_camera_ty_ = camera_ty;
    infrastructure_camera_tz_ = camera_tz;
    infrastructure_wall_height_ = wall_height;
}

void CameraViewer::set_em_decision_buttons_visible(bool visible)
{
    if (em_accept_button_)
    {
        em_accept_button_->setVisible(true);
        em_accept_button_->setEnabled(visible);
    }
    if (em_reject_button_)
    {
        em_reject_button_->setVisible(true);
        em_reject_button_->setEnabled(visible);
    }
}

void CameraViewer::set_synthetic_overlay(const QImage& overlay)
{
    std::lock_guard<std::mutex> lock(wireframe_mutex_);
    synthetic_overlay_ = overlay;
}

// ---------------------------------------------------------------------------
// show / hide → start / stop timer
// ---------------------------------------------------------------------------
void CameraViewer::showEvent(QShowEvent* e)
{
    QDialog::showEvent(e);
    const int interval = 1000 / std::max(1, fps_spin_->value());
    timer_->start(interval);
    std::println("[CameraViewer] showEvent -> timer started (fps={}, interval={}ms)",
                 fps_spin_->value(), interval);
}

void CameraViewer::hideEvent(QHideEvent* e)
{
    timer_->stop();
    std::println("[CameraViewer] hideEvent -> timer stopped");
    QDialog::hideEvent(e);
}

// ---------------------------------------------------------------------------
// grab_frame – called by timer
// ---------------------------------------------------------------------------
void CameraViewer::grab_frame()
{
    static int grab_counter = 0;
    if (++grab_counter % 40 == 1)
        std::println("[CameraViewer] heartbeat: grab_frame running (visible={})", isVisible());

    if (!proxy_)
    {
        status_label_->setText("No proxy");
        std::println("[CameraViewer] ERROR: proxy is null");
        return;
    }

    try
    {
        const auto tdata = proxy_->getAll(false);
        const auto timg = tdata.image;
        int depth_w_cached = 0;
        int depth_h_cached = 0;
        std::vector<float> depth_m_cached;
        const bool have_depth_cached = extract_depth_from_tdata(tdata, depth_w_cached, depth_h_cached, depth_m_cached);

        // ---- RGB --------------------------------------------------------
        if (timg.width <= 0 || timg.height <= 0 || timg.image.empty())
        {
            status_label_->setText("Empty image from getAll()");
            static int empty_log_counter = 0;
            if (++empty_log_counter % 20 == 1)
                std::println("[CameraViewer] empty image: w={} h={} depth={} size={} compressed={}",
                             timg.width, timg.height, timg.depth,
                             static_cast<int>(timg.image.size()), timg.compressed ? 1 : 0);
            return;
        }

        QPixmap px = timage_to_pixmap(timg);
        if (px.isNull())
        {
            std::println("[CameraViewer] DECODE FAILED: w={} h={} depth={} size={} compressed={}",
                         timg.width, timg.height, timg.depth, static_cast<int>(timg.image.size()),
                         timg.compressed ? 1 : 0);
            return;
        }

        if (!raw_mode_)
        {
        {
            std::lock_guard<std::mutex> lock(wireframe_mutex_);
            if (infrastructure_ctx_valid_ && !infrastructure_room_polygon_.empty() && mask_infrastructure_)
            {
                QImage qimg = px.toImage().convertToFormat(QImage::Format_RGB888);
                const int w = qimg.width();
                const int h = qimg.height();

                const float max_reasonable_fx = static_cast<float>(std::max(1, timg.width)) * 5.f;
                const float max_reasonable_fy = static_cast<float>(std::max(1, timg.height)) * 5.f;
                float fx = static_cast<float>(timg.focalx);
                float fy = static_cast<float>(timg.focaly);
                if (!std::isfinite(fx) || fx < 10.f || fx > max_reasonable_fx)
                    fx = static_cast<float>(std::max(1, timg.width)) * 0.9f;
                if (!std::isfinite(fy) || fy < 10.f || fy > max_reasonable_fy)
                    fy = static_cast<float>(std::max(1, timg.height)) * 0.9f;
                const float cx = static_cast<float>(std::max(1, timg.width)) * 0.5f;
                const float cy = static_cast<float>(std::max(1, timg.height)) * 0.5f;
                constexpr float near_depth = 0.05f;
                const InfraMaskMode infra_mode = read_infra_mask_mode();
                const bool include_floor_ceiling = (infra_mode == InfraMaskMode::AllPlanes);

                auto project = [&](const Eigen::Vector3f& p_cam, QPointF& q) -> bool
                {
                    const float depth = p_cam.y(); // strict camera convention: y forward
                    if (depth <= near_depth)
                        return false;
                    const float u = cx + fx * (p_cam.x() / depth);
                    const float v = cy - fy * (p_cam.z() / depth);
                    q = QPointF(u, v);
                    return std::isfinite(u) && std::isfinite(v);
                };

                auto world_to_camera = [&](const Eigen::Vector2f& p_world, float z_world) -> Eigen::Vector3f
                {
                    const Eigen::Vector2f p_robot = infrastructure_robot_pose_.inverse() * p_world;
                    return Eigen::Vector3f(p_robot.x() - infrastructure_camera_tx_,
                                           p_robot.y() - infrastructure_camera_ty_,
                                           z_world - infrastructure_camera_tz_);
                };

                // 1) Build infrastructure mask polygons from visible walls, floor and ceiling.
                QImage infra_mask(w, h, QImage::Format_Grayscale8);
                infra_mask.fill(0);
                std::vector<QPolygonF> wall_polys;

                auto clip_near = [&](const std::vector<Eigen::Vector3f>& in_poly) -> std::vector<Eigen::Vector3f>
                {
                    std::vector<Eigen::Vector3f> out;
                    if (in_poly.empty())
                        return out;
                    out.reserve(in_poly.size() + 2);
                    for (std::size_t j = 0; j < in_poly.size(); ++j)
                    {
                        const Eigen::Vector3f S = in_poly[j];
                        const Eigen::Vector3f E = in_poly[(j + 1) % in_poly.size()];
                        const bool S_in = S.y() > near_depth;
                        const bool E_in = E.y() > near_depth;

                        auto intersect = [&]() -> Eigen::Vector3f
                        {
                            const float denom = E.y() - S.y();
                            if (std::abs(denom) < 1e-8f)
                                return S;
                            const float t = (near_depth - S.y()) / denom;
                            return S + t * (E - S);
                        };

                        if (S_in && E_in)
                            out.push_back(E);
                        else if (S_in && !E_in)
                            out.push_back(intersect());
                        else if (!S_in && E_in)
                        {
                            out.push_back(intersect());
                            out.push_back(E);
                        }
                    }
                    return out;
                };

                {
                    QPainter pm(&infra_mask);
                    pm.setRenderHint(QPainter::Antialiasing, false);
                    pm.setPen(Qt::NoPen);
                    pm.setBrush(Qt::white);

                    for (std::size_t i = 0; i < infrastructure_room_polygon_.size(); ++i)
                    {
                        const Eigen::Vector2f a = infrastructure_room_polygon_[i];
                        const Eigen::Vector2f b = infrastructure_room_polygon_[(i + 1) % infrastructure_room_polygon_.size()];
                        const Eigen::Vector3f a0 = world_to_camera(a, 0.f);
                        const Eigen::Vector3f b0 = world_to_camera(b, 0.f);
                        const Eigen::Vector3f b1 = world_to_camera(b, infrastructure_wall_height_);
                        const Eigen::Vector3f a1 = world_to_camera(a, infrastructure_wall_height_);

                        const std::vector<Eigen::Vector3f> quad = {a0, b0, b1, a1};
                        const auto clipped = clip_near(quad);
                        if (clipped.size() < 3)
                            continue;

                        QPolygonF poly;
                        poly.reserve(static_cast<int>(clipped.size()));
                        for (const auto& p : clipped)
                        {
                            QPointF q;
                            if (project(p, q))
                                poly << q;
                        }
                        if (poly.size() < 3)
                            continue;

                        QRectF bb = poly.boundingRect();
                        if (bb.right() < 0 || bb.left() >= w || bb.bottom() < 0 || bb.top() >= h)
                            continue;

                        wall_polys.push_back(poly);
                        pm.drawPolygon(poly);
                    }

                    if (include_floor_ceiling)
                    {
                        // Floor polygon
                        {
                            std::vector<Eigen::Vector3f> floor_world_cam;
                            floor_world_cam.reserve(infrastructure_room_polygon_.size());
                            for (const auto& p : infrastructure_room_polygon_)
                                floor_world_cam.emplace_back(world_to_camera(p, 0.f));

                            const auto clipped = clip_near(floor_world_cam);
                            if (clipped.size() >= 3)
                            {
                                QPolygonF poly;
                                poly.reserve(static_cast<int>(clipped.size()));
                                for (const auto& p : clipped)
                                {
                                    QPointF q;
                                    if (project(p, q))
                                        poly << q;
                                }
                                if (poly.size() >= 3)
                                {
                                    QRectF bb = poly.boundingRect();
                                    if (!(bb.right() < 0 || bb.left() >= w || bb.bottom() < 0 || bb.top() >= h))
                                    {
                                        pm.drawPolygon(poly);
                                    }
                                }
                            }
                        }

                        // Ceiling polygon
                        {
                            std::vector<Eigen::Vector3f> ceiling_world_cam;
                            ceiling_world_cam.reserve(infrastructure_room_polygon_.size());
                            for (const auto& p : infrastructure_room_polygon_)
                                ceiling_world_cam.emplace_back(world_to_camera(p, infrastructure_wall_height_));

                            const auto clipped = clip_near(ceiling_world_cam);
                            if (clipped.size() >= 3)
                            {
                                QPolygonF poly;
                                poly.reserve(static_cast<int>(clipped.size()));
                                for (const auto& p : clipped)
                                {
                                    QPointF q;
                                    if (project(p, q))
                                        poly << q;
                                }
                                if (poly.size() >= 3)
                                {
                                    QRectF bb = poly.boundingRect();
                                    if (!(bb.right() < 0 || bb.left() >= w || bb.bottom() < 0 || bb.top() >= h))
                                    {
                                        pm.drawPolygon(poly);
                                    }
                                }
                            }
                        }
                    }

                    pm.end();
                }

                std::vector<Eigen::Vector2f> room_robot;
                room_robot.reserve(infrastructure_room_polygon_.size());
                for (const auto& p : infrastructure_room_polygon_)
                    room_robot.emplace_back(infrastructure_robot_pose_.inverse() * p);

                std::vector<Eigen::Vector2f> room_camera_xy;
                room_camera_xy.reserve(room_robot.size());
                for (const auto& p : room_robot)
                    room_camera_xy.emplace_back(p.x() - infrastructure_camera_tx_, p.y() - infrastructure_camera_ty_);

                auto point_in_polygon = [](const Eigen::Vector2f& p, const std::vector<Eigen::Vector2f>& poly) -> bool
                {
                    if (poly.size() < 3)
                        return false;
                    bool inside = false;
                    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
                    {
                        const auto& pi = poly[i];
                        const auto& pj = poly[j];
                        const bool intersect = ((pi.y() > p.y()) != (pj.y() > p.y())) &&
                                               (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) /
                                                            std::max(1e-9f, (pj.y() - pi.y())) + pi.x());
                        if (intersect)
                            inside = !inside;
                    }
                    return inside;
                };

                struct WallPlane
                {
                    Eigen::Vector3f a0;
                    Eigen::Vector3f edge_u;
                    float edge_u_sq;
                    Eigen::Vector3f edge_v;
                    float edge_v_sq;
                    Eigen::Vector3f normal;
                    float plane_d;
                };

                std::vector<WallPlane> wall_planes;
                const float floor_z = -infrastructure_camera_tz_;
                const float ceil_z = infrastructure_wall_height_ - infrastructure_camera_tz_;

                if (room_camera_xy.size() >= 3)
                {
                    wall_planes.reserve(room_camera_xy.size());
                    for (std::size_t e = 0; e < infrastructure_room_polygon_.size(); ++e)
                    {
                        const Eigen::Vector2f aw = infrastructure_room_polygon_[e];
                        const Eigen::Vector2f bw = infrastructure_room_polygon_[(e + 1) % infrastructure_room_polygon_.size()];
                        const Eigen::Vector3f a0 = world_to_camera(aw, 0.f);
                        const Eigen::Vector3f b0 = world_to_camera(bw, 0.f);
                        const Eigen::Vector3f a1 = world_to_camera(aw, infrastructure_wall_height_);

                        const Eigen::Vector3f edge_u = b0 - a0;
                        const Eigen::Vector3f edge_v = a1 - a0;
                        const float edge_u_sq = edge_u.squaredNorm();
                        const float edge_v_sq = edge_v.squaredNorm();
                        if (edge_u_sq < 1e-9f || edge_v_sq < 1e-9f)
                            continue;

                        Eigen::Vector3f normal = edge_u.cross(edge_v);
                        const float n_norm = normal.norm();
                        if (n_norm < 1e-9f)
                            continue;
                        normal /= n_norm;

                        wall_planes.push_back(WallPlane{a0, edge_u, edge_u_sq, edge_v, edge_v_sq, normal, -normal.dot(a0)});
                    }
                }

                auto first_infra_depth_along_ray = [&](const Eigen::Vector3f& ray_cam, int* hit_kind = nullptr) -> float
                {
                    if (room_camera_xy.size() < 3)
                        return std::numeric_limits<float>::infinity();

                    float best_t = std::numeric_limits<float>::infinity();
                    int best_kind = 0;

                    if (include_floor_ceiling && std::abs(ray_cam.z()) > 1e-7f)
                    {
                        const float t_floor = floor_z / ray_cam.z();
                        if (t_floor > near_depth)
                        {
                            const Eigen::Vector2f hit_xy = (t_floor * ray_cam).head<2>();
                            if (point_in_polygon(hit_xy, room_camera_xy))
                            {
                                if (t_floor < best_t)
                                {
                                    best_t = t_floor;
                                    best_kind = 1;
                                }
                            }
                        }

                        const float t_ceil = ceil_z / ray_cam.z();
                        if (t_ceil > near_depth)
                        {
                            const Eigen::Vector2f hit_xy = (t_ceil * ray_cam).head<2>();
                            if (point_in_polygon(hit_xy, room_camera_xy))
                            {
                                if (t_ceil < best_t)
                                {
                                    best_t = t_ceil;
                                    best_kind = 2;
                                }
                            }
                        }
                    }

                    for (const auto& wp : wall_planes)
                    {
                        const float den = wp.normal.dot(ray_cam);
                        if (std::abs(den) < 1e-7f)
                            continue;

                        const float t = -wp.plane_d / den;
                        if (t <= near_depth)
                            continue;

                        const Eigen::Vector3f hit = t * ray_cam;
                        const Eigen::Vector3f rel = hit - wp.a0;
                        const float alpha = rel.dot(wp.edge_u) / wp.edge_u_sq;
                        const float beta = rel.dot(wp.edge_v) / wp.edge_v_sq;
                        if (alpha < -1e-4f || alpha > 1.f + 1e-4f)
                            continue;
                        if (beta < -1e-4f || beta > 1.f + 1e-4f)
                            continue;

                        if (t < best_t)
                        {
                            best_t = t;
                            best_kind = 3;
                        }
                    }

                    if (hit_kind != nullptr)
                        *hit_kind = std::isfinite(best_t) ? best_kind : 0;
                    return best_t;
                };

                // 3) Two-pass masking: mask all infrastructure rays, then restore foreground by depth.
                if (have_depth_cached && depth_w_cached > 0 && depth_h_cached > 0)
                {
                    const QImage original_qimg = qimg.copy();
                    int finite_hits = 0;
                    int floor_hits = 0;
                    int ceil_hits = 0;
                    int wall_hits = 0;

                    for (int y = 0; y < h; ++y)
                    {
                        uchar* rgb = qimg.scanLine(y);
                        const uchar* wm = infra_mask.constScanLine(y);
                        for (int x = 0; x < w; ++x)
                        {
                            const Eigen::Vector3f ray_cam((static_cast<float>(x) + 0.5f - cx) / std::max(1e-5f, fx),
                                                          1.f,
                                                          (cy - (static_cast<float>(y) + 0.5f)) / std::max(1e-5f, fy));
                            int hit_kind = 0;
                            const float d_inf = first_infra_depth_along_ray(ray_cam, &hit_kind);
                            if (std::isfinite(d_inf) || wm[x] > 0)
                            {
                                if (std::isfinite(d_inf))
                                {
                                    ++finite_hits;
                                    if (hit_kind == 1) ++floor_hits;
                                    else if (hit_kind == 2) ++ceil_hits;
                                    else if (hit_kind == 3) ++wall_hits;
                                }
                                rgb[3 * x + 0] = 0;
                                rgb[3 * x + 1] = 0;
                                rgb[3 * x + 2] = 0;
                            }
                        }
                    }

                    int restored_hits = 0;
                    for (int y = 0; y < h; ++y)
                    {
                        uchar* rgb = qimg.scanLine(y);
                        const uchar* rgb0 = original_qimg.constScanLine(y);
                        for (int x = 0; x < w; ++x)
                        {
                            int hit_kind = 0;
                            const Eigen::Vector3f ray_cam((static_cast<float>(x) + 0.5f - cx) / std::max(1e-5f, fx),
                                                          1.f,
                                                          (cy - (static_cast<float>(y) + 0.5f)) / std::max(1e-5f, fy));
                            const float d_inf = first_infra_depth_along_ray(ray_cam, &hit_kind);
                            if (!std::isfinite(d_inf))
                                continue;

                            const int xd = std::clamp((x * depth_w_cached) / std::max(1, w), 0, depth_w_cached - 1);
                            const int yd = std::clamp((y * depth_h_cached) / std::max(1, h), 0, depth_h_cached - 1);
                            const float d_obs = depth_m_cached[static_cast<std::size_t>(yd) * static_cast<std::size_t>(depth_w_cached)
                                                             + static_cast<std::size_t>(xd)];
                            const float margin = 0.08f + 0.01f * d_inf;
                            const bool keep_foreground = (hit_kind == 3) &&
                                                         std::isfinite(d_obs) && d_obs > near_depth &&
                                                         std::isfinite(d_inf) && (d_obs + margin < d_inf);
                            if (keep_foreground)
                            {
                                ++restored_hits;
                                rgb[3 * x + 0] = rgb0[3 * x + 0];
                                rgb[3 * x + 1] = rgb0[3 * x + 1];
                                rgb[3 * x + 2] = rgb0[3 * x + 2];
                            }
                        }
                    }

                    static int mask_diag_counter = 0;
                    if (++mask_diag_counter % 30 == 0)
                    {
                        std::println("[CameraViewer][mask] infra_hits={}/{} wall={} floor={} ceil={} restored={} depth={}x{}",
                                     finite_hits, w * h, wall_hits, floor_hits, ceil_hits, restored_hits,
                                     depth_w_cached, depth_h_cached);
                    }
                }
                else
                {
                    // Fallback when depth is unavailable.
                    for (int y = 0; y < h; ++y)
                    {
                        uchar* rgb = qimg.scanLine(y);
                        const uchar* wm = infra_mask.constScanLine(y);
                        for (int x = 0; x < w; ++x)
                        {
                            const Eigen::Vector3f ray_cam((static_cast<float>(x) + 0.5f - cx) / std::max(1e-5f, fx),
                                                          1.f,
                                                          (cy - (static_cast<float>(y) + 0.5f)) / std::max(1e-5f, fy));
                            if (std::isfinite(first_infra_depth_along_ray(ray_cam)) || wm[x] > 0)
                            {
                                rgb[3 * x + 0] = 0;
                                rgb[3 * x + 1] = 0;
                                rgb[3 * x + 2] = 0;
                            }
                        }
                    }
                }

                // 5) Infrastructure debug polygon drawing intentionally disabled.

                px = QPixmap::fromImage(qimg);
            }
        }

        QPainter painter(&px);
        painter.setRenderHint(QPainter::Antialiasing, true);
        // YOLO mask/polygon painting intentionally disabled in EM validator mode.

        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> wireframe_segments;
        std::vector<QColor> wireframe_colors;
        QString wire_label;
        std::vector<Eigen::Vector3f> annotation_points;
        std::vector<QString> annotation_texts;
        std::vector<QColor> annotation_colors;
        std::vector<Eigen::Vector3f> em_points;
        std::vector<int> em_classes;
        std::vector<QColor> em_colors;
        std::vector<QString> em_labels;
        QString em_title;
        QImage synth_overlay;
        {
            std::lock_guard<std::mutex> lock(wireframe_mutex_);
            wireframe_segments = wireframe_segments_camera_;
            wireframe_colors = wireframe_segment_colors_camera_;
            wire_label = wireframe_label_;
            annotation_points = wireframe_annotation_points_camera_;
            annotation_texts = wireframe_annotation_texts_;
            annotation_colors = wireframe_annotation_colors_;
            em_points = em_points_camera_;
            em_classes = em_point_classes_;
            em_colors = em_class_colors_;
            em_labels = em_class_labels_;
            em_title = em_overlay_title_;
            synth_overlay = synthetic_overlay_;
        }

        if (!wireframe_segments.empty())
        {
            const float fallback_fx = static_cast<float>(timg.width) * 0.9f;
            const float fallback_fy = static_cast<float>(timg.height) * 0.9f;
            const float max_reasonable_fx = static_cast<float>(timg.width) * 5.f;
            const float max_reasonable_fy = static_cast<float>(timg.height) * 5.f;

            float fx = static_cast<float>(timg.focalx);
            float fy = static_cast<float>(timg.focaly);
            const bool bad_fx = !std::isfinite(fx) || fx < 10.f || fx > max_reasonable_fx;
            const bool bad_fy = !std::isfinite(fy) || fy < 10.f || fy > max_reasonable_fy;
            if (bad_fx) fx = fallback_fx;
            if (bad_fy) fy = fallback_fy;

            const float cx = static_cast<float>(timg.width) * 0.5f;
            const float cy = static_cast<float>(timg.height) * 0.5f;
            constexpr float near_depth = 0.05f;

            enum class ProjectionMode { DepthOnY, DepthOnX };

            auto project = [&](const Eigen::Vector3f& p, ProjectionMode mode, QPointF& q) -> bool
            {
                const float depth = (mode == ProjectionMode::DepthOnY) ? p.y() : p.x();
                if (depth <= near_depth)
                    return false;

                const float lateral = (mode == ProjectionMode::DepthOnY) ? p.x() : p.y();
                const float u = cx + fx * (lateral / depth);
                const float v = cy - fy * (p.z() / depth);
                q = QPointF(u, v);
                return true;
            };

            auto draw_segment_clipped = [&](Eigen::Vector3f a, Eigen::Vector3f b, ProjectionMode mode) -> bool
            {
                const float a_depth = (mode == ProjectionMode::DepthOnY) ? a.y() : a.x();
                const float b_depth = (mode == ProjectionMode::DepthOnY) ? b.y() : b.x();
                const bool a_behind = a_depth <= near_depth;
                const bool b_behind = b_depth <= near_depth;
                if (a_behind && b_behind)
                    return false;

                if (a_behind != b_behind)
                {
                    const float denom = b_depth - a_depth;
                    if (std::abs(denom) < 1e-6f)
                        return false;
                    const float t = (near_depth - a_depth) / denom;
                    const Eigen::Vector3f p_clip = a + t * (b - a);
                    if (a_behind)
                        a = p_clip;
                    else
                        b = p_clip;
                }

                QPointF qa, qb;
                if (!project(a, mode, qa) || !project(b, mode, qb))
                    return false;
                painter.drawLine(qa, qb);
                return true;
            };

            auto count_visible_segments = [&](ProjectionMode mode) -> int
            {
                int count = 0;
                for (const auto& seg : wireframe_segments)
                {
                    const float a_depth = (mode == ProjectionMode::DepthOnY) ? seg.first.y() : seg.first.x();
                    const float b_depth = (mode == ProjectionMode::DepthOnY) ? seg.second.y() : seg.second.x();
                    if (a_depth > near_depth || b_depth > near_depth)
                        ++count;
                }
                return count;
            };

            const int vis_y = count_visible_segments(ProjectionMode::DepthOnY);
            const int vis_x = count_visible_segments(ProjectionMode::DepthOnX);
            const ProjectionMode best_mode = (vis_x > vis_y) ? ProjectionMode::DepthOnX
                                                             : ProjectionMode::DepthOnY;

            for (std::size_t si = 0; si < wireframe_segments.size(); ++si)
            {
                const QColor seg_color = (si < wireframe_colors.size()) ? wireframe_colors[si] : QColor(255, 220, 60);
                painter.setPen(QPen(seg_color, 2));
                const auto& seg = wireframe_segments[si];
                draw_segment_clipped(seg.first, seg.second, best_mode);
            }

            for (std::size_t ai = 0; ai < annotation_points.size() && ai < annotation_texts.size(); ++ai)
            {
                QPointF p;
                if (!project(annotation_points[ai], best_mode, p))
                    continue;
                const QColor c = (ai < annotation_colors.size()) ? annotation_colors[ai] : QColor(255, 255, 255);
                painter.setPen(QPen(c, 1));
                painter.drawText(p + QPointF(6.0, -6.0), annotation_texts[ai]);
            }

            if (!wire_label.isEmpty())
                painter.drawText(QPoint(12, 22), wire_label);
        }

        if (!em_points.empty() && em_points.size() == em_classes.size())
        {
            const float fallback_fx = static_cast<float>(timg.width) * 0.9f;
            const float fallback_fy = static_cast<float>(timg.height) * 0.9f;
            const float max_reasonable_fx = static_cast<float>(timg.width) * 5.f;
            const float max_reasonable_fy = static_cast<float>(timg.height) * 5.f;

            float fx = static_cast<float>(timg.focalx);
            float fy = static_cast<float>(timg.focaly);
            const bool bad_fx = !std::isfinite(fx) || fx < 10.f || fx > max_reasonable_fx;
            const bool bad_fy = !std::isfinite(fy) || fy < 10.f || fy > max_reasonable_fy;
            if (bad_fx) fx = fallback_fx;
            if (bad_fy) fy = fallback_fy;

            const float cx = static_cast<float>(timg.width) * 0.5f;
            const float cy = static_cast<float>(timg.height) * 0.5f;
            constexpr float near_depth = 0.05f;

            enum class ProjectionMode { DepthOnY, DepthOnX };
            auto visible_count = [&](ProjectionMode mode) -> int
            {
                int count = 0;
                for (const auto& p : em_points)
                {
                    const float depth = (mode == ProjectionMode::DepthOnY) ? p.y() : p.x();
                    if (depth > near_depth) ++count;
                }
                return count;
            };
            const ProjectionMode mode = (visible_count(ProjectionMode::DepthOnX) > visible_count(ProjectionMode::DepthOnY))
                ? ProjectionMode::DepthOnX
                : ProjectionMode::DepthOnY;

            auto project = [&](const Eigen::Vector3f& p, QPointF& q) -> bool
            {
                const float depth = (mode == ProjectionMode::DepthOnY) ? p.y() : p.x();
                if (depth <= near_depth) return false;
                const float lateral = (mode == ProjectionMode::DepthOnY) ? p.x() : p.y();
                q = QPointF(cx + fx * (lateral / depth), cy - fy * (p.z() / depth));
                return true;
            };

            const bool aa_prev = painter.testRenderHint(QPainter::Antialiasing);
            painter.setRenderHint(QPainter::Antialiasing, false);
            painter.setPen(Qt::NoPen);
            for (std::size_t i = 0; i < em_points.size(); ++i)
            {
                QPointF pix;
                if (!project(em_points[i], pix))
                    continue;

                const int cls = em_classes[i];
                QColor c(255, 255, 255, 220);
                if (cls >= 0 && cls < static_cast<int>(em_colors.size()))
                    c = em_colors[cls];
                c.setAlpha(255);
                painter.setBrush(c);
                painter.drawRect(QRectF(pix.x() - 1.0, pix.y() - 1.0, 2.0, 2.0));
            }
            painter.setRenderHint(QPainter::Antialiasing, aa_prev);

            // Redraw object wireframe on top of class-colored points for per-iteration EM readability.
            if (!wireframe_segments.empty())
            {
                auto project_seg = [&](const Eigen::Vector3f& p, QPointF& q) -> bool
                {
                    const float depth = (mode == ProjectionMode::DepthOnY) ? p.y() : p.x();
                    if (depth <= near_depth) return false;
                    const float lateral = (mode == ProjectionMode::DepthOnY) ? p.x() : p.y();
                    q = QPointF(cx + fx * (lateral / depth), cy - fy * (p.z() / depth));
                    return std::isfinite(q.x()) && std::isfinite(q.y());
                };

                for (std::size_t si = 0; si < wireframe_segments.size(); ++si)
                {
                    const QColor seg_color = (si < wireframe_colors.size()) ? wireframe_colors[si] : QColor(255, 220, 60);
                    painter.setPen(QPen(seg_color, 2));
                    QPointF q0, q1;
                    if (project_seg(wireframe_segments[si].first, q0) && project_seg(wireframe_segments[si].second, q1))
                        painter.drawLine(q0, q1);
                }
            }

            painter.setPen(QPen(QColor(255, 255, 255), 1));
            if (!em_title.isEmpty())
                painter.drawText(QPoint(12, 42), em_title);

            const int legend_count = std::min(static_cast<int>(em_labels.size()), 6);
            for (int k = 0; k < legend_count; ++k)
            {
                const int y = 62 + k * 18;
                QColor c = (k < static_cast<int>(em_colors.size())) ? em_colors[k] : QColor(255, 255, 255);
                c.setAlpha(255);
                painter.fillRect(QRect(14, y - 11, 14, 14), c);
                painter.setPen(QPen(QColor(255, 255, 255), 1));
                painter.drawRect(QRect(14, y - 11, 14, 14));
                painter.setPen(QPen(QColor(245, 245, 245), 1));
                painter.drawText(QPoint(30, y), em_labels[k]);
            }
        }
        // Composite synthetic camera overlay (walls/floor/furniture wireframes)
        if (!synth_overlay.isNull() && synth_overlay.size() == px.size())
        {
            painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
            painter.drawImage(0, 0, synth_overlay);
        }
        painter.end();
        } // !raw_mode_

        rgb_label_->setPixmap(
            px.scaled(rgb_label_->size(), Qt::KeepAspectRatio,
                      Qt::FastTransformation));

        if (tabs_->currentIndex() == 1)
        {
            const bool have_depth = have_depth_cached;

            if (have_depth && depth_w_cached > 0 && depth_h_cached > 0)
            {
                QImage depth_img = depth_colormap_image(depth_m_cached, depth_w_cached, depth_h_cached);
                depth_label_->setPixmap(QPixmap::fromImage(depth_img).scaled(
                    depth_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            }
            else
            {
                depth_label_->setText("No depth frame available");
            }
        }

        // ---- FPS display ------------------------------------------------
        ++frame_count_;
        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        if (now - last_fps_tick_ >= 1000)
        {
            status_label_->setText(
                QString("%1 fps  |  %2 × %3  |  objs: %4")
                    .arg(frame_count_)
                    .arg(timg.width)
                    .arg(timg.height)
                    .arg(static_cast<int>(tdata.objects.size())));
            frame_count_  = 0;
            last_fps_tick_ = now;
        }
    }
    catch (const Ice::Exception& ex)
    {
        status_label_->setText(QString("Ice error: %1").arg(ex.what()));
        std::println("[CameraViewer] ICE exception: {}", ex.what());
    }
    catch (const std::exception& ex)
    {
        status_label_->setText(QString("Error: %1").arg(ex.what()));
        std::println("[CameraViewer] std exception: {}", ex.what());
    }
    catch (...)
    {
        status_label_->setText("Error: unknown exception in camera proxy");
        std::println("[CameraViewer] unknown exception in grab_frame");
    }
}

// ---------------------------------------------------------------------------
// timage_to_pixmap – handles both compressed (JPEG) and raw RGB/BGR
// ---------------------------------------------------------------------------
QPixmap CameraViewer::timage_to_pixmap(const RoboCompImageSegmentation::TImage& img)
{
    if (img.image.empty() || img.width <= 0 || img.height <= 0) return {};

    if (img.compressed)
    {
        // Encoded bytes (JPEG/PNG/...) with auto-detected format
        QImage qi;
        if (qi.loadFromData(reinterpret_cast<const uchar*>(img.image.data()),
                            static_cast<int>(img.image.size())))
            return QPixmap::fromImage(qi);
        // If compressed flag is wrong, continue with raw decode path.
    }

    // Raw bytes: if depth is invalid/corrupted, infer from payload size.
    const int channels = img.depth;
    if (channels != 1 and channels != 3 and channels != 4)
        return {};

    const std::size_t expected_size = static_cast<std::size_t>(img.width)
                                    * static_cast<std::size_t>(img.height)
                                    * static_cast<std::size_t>(channels);
    if (img.image.size() < expected_size)
        return {};

    QImage::Format fmt = QImage::Format_RGB888;
    if      (channels == 4) fmt = QImage::Format_RGBA8888;
    else if (channels == 1) fmt = QImage::Format_Grayscale8;

    QImage qi(reinterpret_cast<const uchar*>(img.image.data()),
              img.width, img.height, img.width * channels, fmt);
    return QPixmap::fromImage(qi.copy()); // .copy() detaches from raw buffer
}
