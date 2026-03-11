#pragma once

#include <QDialog>
#include <QLabel>
#include <QTimer>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStatusBar>
#include <QComboBox>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QColor>
#include <QPushButton>
#include <QShowEvent>
#include <QHideEvent>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <mutex>

#include <ImageSegmentation.h>

/**
 * CameraViewer – popup dialog that continuously fetches and displays RGB
 * (and optionally depth) frames from a CameraRGBDSimple proxy.
 *
 * Usage:
 *   camera_viewer_ = new CameraViewer(imagesegmentation_proxy, this);
 *   connect(pushButton_camera, &QPushButton::toggled,
 *           camera_viewer_,    &CameraViewer::setVisible);
 *   connect(camera_viewer_, &QDialog::finished,
 *           pushButton_camera, [btn](int){ btn->setChecked(false); });
 */
class CameraViewer : public QDialog
{
    Q_OBJECT
    public:
        explicit CameraViewer(RoboCompImageSegmentation::ImageSegmentationPrxPtr proxy,
                            QWidget* parent = nullptr);

        /// Frame-grab interval in milliseconds (default 100 ms → ~10 fps).
        void set_period_ms(int ms);

        /// Set 3D wireframe segments in camera frame (x=lateral, y=forward, z=up).
        void set_wireframe_segments_camera(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& segments,
                           const QString& label);

        /// Set 3D wireframe segments with per-segment color.
        void set_wireframe_segments_camera_colored(
            const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& segments,
            const std::vector<QColor>& segment_colors,
            const QString& label);

        /// Set text annotations to be drawn close to wireframes.
        void set_wireframe_annotations_camera(const std::vector<Eigen::Vector3f>& anchor_points_camera,
                                              const std::vector<QString>& annotation_texts,
                                              const std::vector<QColor>& annotation_colors);

        /// Clear wireframe overlay.
        void clear_wireframe_overlay();

        /// Set EM point overlay in camera frame with per-point class assignment.
        void set_em_points_overlay(const std::vector<Eigen::Vector3f>& points_camera,
                       const std::vector<int>& point_classes,
                       const std::vector<QColor>& class_colors,
                       const std::vector<QString>& class_labels,
                       const QString& title);

        /// Clear EM point overlay.
        void clear_em_points_overlay();

        /// Update geometric context to suppress known infrastructure (floor/walls) pixels.
        void set_infrastructure_context(const Eigen::Affine2f& robot_pose,
                        const std::vector<Eigen::Vector2f>& room_polygon,
                        float camera_tx,
                        float camera_ty,
                        float camera_tz,
                        float wall_height = 2.5f);

    signals:
        void emRequested();

    protected:
        void showEvent(QShowEvent* e) override;
        void hideEvent(QHideEvent* e) override;

    private slots:
        void grab_frame();

    private:
        RoboCompImageSegmentation::ImageSegmentationPrxPtr proxy_;

        // Widgets
        QTabWidget* tabs_         = nullptr;
        QLabel*     rgb_label_    = nullptr;
        QLabel*     depth_label_  = nullptr;
        QLabel*     status_label_ = nullptr;
        QSpinBox*   fps_spin_     = nullptr;
        QPushButton* em_button_   = nullptr;
        QCheckBox*  depth_check_  = nullptr;

        QTimer* timer_            = nullptr;
        int     frame_count_      = 0;
        qint64  last_fps_tick_    = 0;

        std::mutex wireframe_mutex_;
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> wireframe_segments_camera_;
        std::vector<QColor> wireframe_segment_colors_camera_;
        QString wireframe_label_;
        std::vector<Eigen::Vector3f> wireframe_annotation_points_camera_;
        std::vector<QString> wireframe_annotation_texts_;
        std::vector<QColor> wireframe_annotation_colors_;

        std::vector<Eigen::Vector3f> em_points_camera_;
        std::vector<int> em_point_classes_;
        std::vector<QColor> em_class_colors_;
        std::vector<QString> em_class_labels_;
        QString em_overlay_title_;

        bool infrastructure_ctx_valid_ = false;
        Eigen::Affine2f infrastructure_robot_pose_ = Eigen::Affine2f::Identity();
        std::vector<Eigen::Vector2f> infrastructure_room_polygon_;
        float infrastructure_camera_tx_ = 0.f;
        float infrastructure_camera_ty_ = 0.f;
        float infrastructure_camera_tz_ = 0.9f;
        float infrastructure_wall_height_ = 2.5f;

        static QPixmap timage_to_pixmap(const RoboCompImageSegmentation::TImage& img);
};
