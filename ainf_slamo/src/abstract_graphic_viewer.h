//
// Created by pbustos on 19/9/21.
//

#ifndef GIRAFF_VIEWER_ABSTRACT_GRAPHIC_VIEWER_H
#define GIRAFF_VIEWER_ABSTRACT_GRAPHIC_VIEWER_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QOpenGLWidget>
#include <QScrollBar>
#include <QApplication>
#include <QVBoxLayout>
#include <QGraphicsPolygonItem>
#include <QLabel>
#include <iostream>


class AbstractGraphicViewer : public QGraphicsView
{
    Q_OBJECT
    private:
        qreal m_scaleX, m_scaleY;
        QGraphicsItem *robot_polygon;
        QGraphicsEllipseItem *laser_in_robot_sr;
        QLabel *status_label_ = nullptr;  // overlay label (top band)

    public:
        AbstractGraphicViewer(QWidget *parent, QRectF dim_, bool draw_axis = true);
        // laser_x_offset and laser_y_offset are expressed as a fraction of robot size (unitless).
        // Example: laser_y_offset=0.2 places the marker at 20% of robot length along +Y in robot frame.
        std::tuple<QGraphicsItem*, QGraphicsEllipseItem*> add_robot(float robot_width,
                                                                           float robot_length,
                                                                           float laser_x_offset = 0,
                                                                           float laser_y_offset= 0.0,
                                                                           QColor color= QColor("Blue"));
        void draw_contour();
        QGraphicsScene scene;
        QGraphicsItem* robot_poly();
        QGraphicsEllipseItem* laser_in_robot();
        void fitToScene(QRectF rect);  // Fit view to show the given rect
        void set_status_text(const QString &text);

    Q_SIGNALS:
      void new_mouse_coordinates(QPointF);
      void right_click(QPointF);
      void robot_moved(QPointF);  // Shift+Left click to reposition robot
      void robot_drag_start(QPointF);  // Left click drag start
      void robot_dragging(QPointF);    // Left click dragging
      void robot_drag_end(QPointF);    // Left click drag end
      void robot_rotate(QPointF);      // Ctrl+Left click to rotate robot toward point

    protected:
        bool _pan = false;
        bool _robot_dragging = false;  // Track robot drag state
        int _panStartX, _panStartY;
        virtual void wheelEvent(QWheelEvent *event);
        virtual void resizeEvent(QResizeEvent *e);
        virtual void mouseMoveEvent(QMouseEvent *event);
        virtual void mousePressEvent(QMouseEvent *event);
        virtual void mouseReleaseEvent(QMouseEvent *event);
};
#endif //GIRAFF_VIEWER_ABSTRACT_GRAPHIC_VIEWER_H
