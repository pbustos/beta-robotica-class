#include "specificworker.h"

#include <algorithm>
#include <cmath>
#include <future>
#include <thread>

void SpecificWorker::display_gt_error(const Eigen::Affine2f &estimated_pose,
                                      const std::optional<Eigen::Affine2f> &gt_pose_opt)
{
    // If GT pose is not available (webots not connected), blank the displays
    if (!gt_pose_opt.has_value())
    {
        lcdNumber_gt_xy_err->display(0.0);
        lcdNumber_gt_ang_err->display(0.0);
        lcdNumber_gt_xy_err->setStyleSheet("background-color: #888888;");  // Grey = no data
        lcdNumber_gt_ang_err->setStyleSheet("background-color: #888888;");
        return;
    }

    // Transform raw Webots pose to map frame using calibration offset
    const Eigen::Affine2f gt = gt_offset_ * gt_pose_opt.value();

    // Position error in cm
    const float dx = estimated_pose.translation().x() - gt.translation().x();
    const float dy = estimated_pose.translation().y() - gt.translation().y();
    const float xy_err_cm = std::sqrt(dx*dx + dy*dy) * 100.f;

    // Angular error in degrees (wrap to [-180, 180])
    const float est_ang = std::atan2(estimated_pose.linear()(1,0), estimated_pose.linear()(0,0));
    const float gt_ang  = std::atan2(gt.linear()(1,0), gt.linear()(0,0));
    float ang_err_deg = qRadiansToDegrees(est_ang - gt_ang);
    while (ang_err_deg >  180.f) ang_err_deg -= 360.f;
    while (ang_err_deg < -180.f) ang_err_deg += 360.f;
    ang_err_deg = std::abs(ang_err_deg);

    lcdNumber_gt_xy_err->display(static_cast<double>(xy_err_cm));
    lcdNumber_gt_ang_err->display(static_cast<double>(ang_err_deg));

    static QString last_xy_color, last_ang_color;
    auto set_if_changed = [](QLCDNumber *w, const QString &style, QString &last) {
        if (style != last) { w->setStyleSheet(style); last = style; }
    };

    QString xy_color;
    if      (xy_err_cm < 5.f)  xy_color = "background-color: #90EE90;";
    else if (xy_err_cm < 15.f) xy_color = "background-color: #FFFF00;";
    else                        xy_color = "background-color: #FF6B6B;";
    set_if_changed(lcdNumber_gt_xy_err, xy_color, last_xy_color);

    QString ang_color;
    if      (ang_err_deg < 3.f)  ang_color = "background-color: #90EE90;";
    else if (ang_err_deg < 10.f) ang_color = "background-color: #FFFF00;";
    else                          ang_color = "background-color: #FF6B6B;";
    set_if_changed(lcdNumber_gt_ang_err, ang_color, last_ang_color);
}

void SpecificWorker::calibrate_gt_offset(const Eigen::Affine2f &estimated_pose, const Eigen::Affine2f &webots_pose)
{
    // Compute offset T such that:  T * webots_pose ≈ estimated_pose
    // => T = estimated_pose * webots_pose.inverse()
    gt_offset_ = estimated_pose * webots_pose.inverse();
    gt_calibrated_ = true;

    const float tx = gt_offset_.translation().x();
    const float ty = gt_offset_.translation().y();
    const float ang = std::atan2(gt_offset_.linear()(1,0), gt_offset_.linear()(0,0));
    qInfo() << "GT calibrated: offset = (" << tx << "," << ty << ") rot =" << qRadiansToDegrees(ang) << "°";
}

void SpecificWorker::slot_calibrate_gt()
{
    if (!loc_initialized_.load())
    {
        qWarning() << "Cannot calibrate GT: room_ai not initialized";
        return;
    }

    // Read current GT pose from buffer
    const auto timestamp = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto &[robot_pose_gt_, lidar_local_, lidar_low_unused_] = buffer_sync.read(timestamp);
    if (!robot_pose_gt_.has_value())
    {
        qWarning() << "Cannot calibrate GT: no Webots pose available";
        return;
    }

    // Get the current estimated pose (thread-safe)
    const auto state = get_loc_state();
    Eigen::Affine2f estimated;
    estimated.setIdentity();
    estimated.translation() = Eigen::Vector2f(state[2], state[3]);
    estimated.linear() = Eigen::Rotation2Df(state[4]).toRotationMatrix();

    calibrate_gt_offset(estimated, robot_pose_gt_.value());
}

void SpecificWorker::read_lidar()
{
    auto wait_period = std::chrono::milliseconds (getPeriod("Compute"));
    while(!stop_lidar_thread)
    {
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        try
        {
            // Get robot GT pose from Webots (only for debug/stats, not used by algorithm)
            if (params.USE_WEBOTS)
            {
                const auto &[position, orientation] = webots2robocomp_proxy->getObjectPose("shadow");
                Eigen::Affine2f eig_pose;
                eig_pose.translation() = Eigen::Vector2f(-position.y/1000.f, position.x/1000.f);
                eig_pose.linear() = Eigen::Rotation2Df(yawFromQuaternion(orientation)).toRotationMatrix();
                buffer_sync.put<0>(std::move(eig_pose), timestamp);
            }

            const float body_offset_sq = params.ROBOT_SEMI_WIDTH * params.ROBOT_SEMI_WIDTH;

            // ---- Launch both lidar requests in parallel (async Ice) — HELIOS ----
            std::future<RoboCompLidar3D::TData> future_high;
            try
            {
                future_high = lidar3d_proxy->getLidarDataWithThreshold2dAsync(
                        params.LIDAR_NAME_HIGH,
                        params.MAX_LIDAR_HIGH_RANGE * 1000.f,
                        params.LIDAR_LOW_DECIMATION_FACTOR);
            }
            catch (const Ice::Exception &e)
            { qWarning() << "[read_lidar] HELIOS async launch failed:" << e.what(); }

            // BPEARL
            std::future<RoboCompLidar3D::TData> future_low;
            try
            {
                future_low = lidar3d1_proxy->getLidarDataWithThreshold2dAsync(
                    params.LIDAR_NAME_LOW,
                    params.MAX_LIDAR_LOW_RANGE * 1000.f,
                    params.LIDAR_LOW_DECIMATION_FACTOR_LOW);
            }
            catch (const Ice::Exception &e)
            { qWarning() << "[read_lidar] BPEARL async launch failed:" << e.what(); }

            // ---- Wait and process HELIOS ----
            const auto data_high = future_high.get();
            std::vector<Eigen::Vector3f> points_high, points_low_high;
            points_low_high.reserve(data_high.points.size());
            points_high.reserve(data_high.points.size());
            for (const auto &p : data_high.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if (pmx*pmx + pmy*pmy > body_offset_sq && pmz < params.LIDAR_HIGH_MAX_HEIGHT)
                {
                    points_low_high.emplace_back(pmx, pmy, pmz);
                    if (pmz > params.LIDAR_HIGH_MIN_HEIGHT)
                        points_high.emplace_back(pmx, pmy, pmz);
                }
            }
            buffer_sync.put<1>(std::make_pair(std::move(points_high), data_high.timestamp), timestamp);

            // ---- Wait and process BPEARL ----
            const auto data_low = future_low.get();
            points_low_high.reserve(points_low_high.size() + data_low.points.size());
            for (const auto &p : data_low.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if (pmx*pmx + pmy*pmy > body_offset_sq)
                    points_low_high.emplace_back(pmx, pmy, pmz);
            }
            buffer_sync.put<2>(std::make_pair(std::move(points_low_high), data_low.timestamp), timestamp);

            // Adjust period with hysteresis
            if (wait_period > std::chrono::milliseconds((long) data_high.period + 2)) --wait_period;
            else if (wait_period < std::chrono::milliseconds((long) data_high.period - 2)) ++wait_period;
            std::this_thread::sleep_for(wait_period);
        }
        catch (const Ice::Exception &e)
        { qWarning() << "Error reading from Lidar3D or robot pose:" << e.what(); }
    }
} // Thread to read the lidar

float SpecificWorker::estimate_orientation_from_points(const std::vector<Eigen::Vector3f> &pts) const
{
    if(pts.size() < 10)
        return 0.f;

    // PCA on 2D points to get dominant direction
    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    for(const auto &p : pts) mean += p.head(2);
    mean /= static_cast<float>(pts.size());

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for(const auto &p : pts)
    {
        const Eigen::Vector2f d = p.head(2) - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(pts.size());

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
    const Eigen::Vector2f v = es.eigenvectors().col(1);  // largest eigenvalue
    return std::atan2(v.y(), v.x());
}

auto SpecificWorker::draw_lidar_points(const std::vector<Eigen::Vector3f> &points_high,
                                       const std::vector<Eigen::Vector3f> &points_low,
                                       const Eigen::Affine2f &robot_pose) -> void
{
    // ---- Helper lambda to draw a point set with a given color pool ----
    auto draw_layer = [&](const std::vector<Eigen::Vector3f> &points,
                          std::vector<QGraphicsEllipseItem*> &pool,
                          const QColor &color, int max_points)
    {
        static const qreal radius_px = 1.5;
        const QRectF ellipse_rect(-radius_px, -radius_px, 2*radius_px, 2*radius_px);
        QPen pen(color); pen.setWidthF(0.0); pen.setCosmetic(true);
        QBrush brush(color);

        const int stride = std::max(1, static_cast<int>(points.size() / max_points));
        const size_t num_draw = (points.size() + stride - 1) / stride;

        // Shrink pool if needed
        while (pool.size() > num_draw)
        {
            auto *p = pool.back();
            viewer->scene.removeItem(p);
            delete p;
            pool.pop_back();
        }

        size_t idx = 0;
        for (size_t i = 0; i < points.size() && idx < num_draw; i += stride, ++idx)
        {
            const Eigen::Vector2f pr = points[i].head<2>();
            const Eigen::Vector2f pw = robot_pose.linear() * pr + robot_pose.translation();

            if (idx < pool.size())
            {
                pool[idx]->setPos(pw.x(), pw.y());
            }
            else
            {
                auto *item = viewer->scene.addEllipse(ellipse_rect, pen, brush);
                item->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
                item->setPos(pw.x(), pw.y());
                item->setZValue(5);
                pool.push_back(item);
            }
        }
    };

    // HELIOS (high) in green
    static std::vector<QGraphicsEllipseItem*> pool_high;
    draw_layer(points_high, pool_high, QColor("Green"), params.MAX_LIDAR_DRAW_POINTS);

    // BPEARL (low) in cyan — fewer points to keep it lightweight
    static std::vector<QGraphicsEllipseItem*> pool_low;
    draw_layer(points_low, pool_low, QColor("Cyan"), params.MAX_LIDAR_DRAW_POINTS / 2);
}

float SpecificWorker::yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat)
{
    double w = quat.w;
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;
    const auto norm = std::sqrt(w*w + x*x + y*y + z*z);
    w /= norm; x /= norm; y /= norm; z /= norm;
    return static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
}
