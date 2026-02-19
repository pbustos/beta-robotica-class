//
// Created by pbustos on 27/10/22.
//

#include "abstract_graphic_viewer.h"
AbstractGraphicViewer::AbstractGraphicViewer(QWidget *parent, QRectF dim_, bool draw_axis)
{
    QVBoxLayout *vlayout = new QVBoxLayout(parent);
    vlayout->addWidget(this);
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    // Set a very large scene rect to allow unlimited panning
    scene.setSceneRect(-100000, -100000, 200000, 200000);
    //scene.setSceneRect(-100, -100, 200, 200);
    this->setScene(&scene);
    this->setCacheMode(QGraphicsView::CacheBackground);
    this->setViewport(new QOpenGLWidget());
    this->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    this->setRenderHint(QPainter::Antialiasing);
    this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    this->scale(1, -1);
    this->setMouseTracking(true);
    // Don't use fitInView - it limits panning. Use centerOn instead.
    this->centerOn(dim_.center());
    this->viewport()->setMouseTracking(true);
    this->setDragMode(QGraphicsView::NoDrag);  // Disable default drag to use custom pan
    this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    // axis
    if(draw_axis)
    {
        // Scale axis length with the provided initial dimensions (scene units).
        // Use a fraction of the room size so it's independent of absolute units.
        const qreal axis_len = std::max<qreal>(1.0, 0.2 * std::min(dim_.width(), dim_.height()));
        const QPointF c = dim_.center();
        QLineF x_axis(c, c + QPointF(axis_len, 0));
        QLineF y_axis(c, c + QPointF(0, axis_len));
        scene.addLine(x_axis, QPen(QColor("Red"), 0.03));
        scene.addLine(y_axis, QPen(QColor("Green"), 0.03));
    }
    this->adjustSize();
}

std::tuple<QGraphicsItem*, QGraphicsEllipseItem*> AbstractGraphicViewer::add_robot(float robot_width, float robot_length,
                                                                   float laser_x_offset, float laser_y_offset, QColor color)
{
    const float sl = robot_length / 2.f;
    const float sw = robot_width / 2.f;
    const QRectF r_poly(-sl, -sw, robot_length, robot_width);
    const QBrush brush(color, Qt::SolidPattern);

    // Use an explicit pen to keep corners sharp even under rotation/zoom.
    QPen pen(color);
    pen.setJoinStyle(Qt::MiterJoin);
    pen.setCapStyle(Qt::SquareCap);
    pen.setWidthF(0.0);          // hairline in device pixels
    pen.setCosmetic(true);       // keep border width constant regardless of view scaling

    robot_polygon = scene.addRect(r_poly, pen, brush);
    robot_polygon->setTransformOriginPoint(r_poly.center());

    // Laser marker: size is in scene units (meters). Make it proportional to robot width.
    const qreal laser_diameter_m = static_cast<qreal>(robot_width) / 10.0;
    const qreal laser_radius_m = laser_diameter_m / 2.0;

    laser_in_robot_sr = new QGraphicsEllipseItem(-laser_radius_m, -laser_radius_m,
                                                laser_diameter_m, laser_diameter_m, robot_polygon);
    laser_in_robot_sr->setBrush(QBrush(QColor("White")));
    QPen laser_pen(QColor("Black"));
    laser_pen.setWidthF(0.0);
    laser_pen.setCosmetic(true);
    laser_in_robot_sr->setPen(laser_pen);

    // Offsets provided as percentage (unitless) of robot size.
    // +X: robot width axis, +Y: robot length axis.
    laser_in_robot_sr->setPos(laser_x_offset * robot_width, laser_y_offset * robot_length);

    robot_polygon->setZValue(55);
    robot_polygon->setPos(0, 0);
    return std::make_tuple(robot_polygon, laser_in_robot_sr);
}
void AbstractGraphicViewer::draw_contour()
{
    auto r = sceneRect();
    auto sr = scene.addRect(r, QPen(QColor("Gray"), 0.1));
    sr->setZValue(15);
}
QGraphicsItem* AbstractGraphicViewer::robot_poly()
{
    return robot_polygon;
}
QGraphicsEllipseItem*  AbstractGraphicViewer::laser_in_robot()
{
    return laser_in_robot_sr;
}
void AbstractGraphicViewer::wheelEvent(QWheelEvent *event)
{
    qreal factor;
    if (event->angleDelta().y() > 0)
        factor = 1.1;
    else
        factor = 0.9;
    auto view_pos = event->position();
    auto scene_pos = this->mapToScene(view_pos.toPoint());
    this->centerOn(scene_pos);
    this->scale(factor, factor);
    auto delta = this->mapToScene(view_pos.toPoint()) - this->mapToScene(this->viewport()->rect().center());
    this->centerOn(scene_pos - delta);
}
void AbstractGraphicViewer::resizeEvent(QResizeEvent *e)
{
    QGraphicsView::resizeEvent(e);
}
void AbstractGraphicViewer::mouseMoveEvent(QMouseEvent *event)
{
    if (_pan)
    {
        // Calculate delta in scene coordinates
        QPointF oldPos = mapToScene(_panStartX, _panStartY);
        QPointF newPos = mapToScene(event->position().toPoint());
        QPointF delta = oldPos - newPos;

        // Update pan start position
        _panStartX = event->position().x();
        _panStartY = event->position().y();

        // Move the view center by delta (keeps scene coordinate system intact)
        QPointF center = mapToScene(viewport()->rect().center());
        centerOn(center + delta);

        event->accept();
        return;
    }

    // Robot dragging (Left button held down)
    if (_robot_dragging && (event->buttons() & Qt::LeftButton))
    {
        auto cursor_in_scene = this->mapToScene(event->position().toPoint());

        if (event->modifiers() & Qt::ControlModifier)
        {
            // Ctrl+drag = rotate robot toward cursor
            emit robot_rotate(cursor_in_scene);
        }
        else
        {
            // Normal drag = move robot
            emit robot_dragging(cursor_in_scene);
        }
        event->accept();
        return;
    }

    QGraphicsView::mouseMoveEvent(event);
}
void AbstractGraphicViewer::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        // Ctrl+Right click = emit right_click signal (cancel target)
        if (event->modifiers() & Qt::ControlModifier)
        {
            auto cursor_in_scene = this->mapToScene(QPoint(event->position().x(), event->position().y()));
            emit right_click(cursor_in_scene);
            event->accept();
            return;
        }
        // Right click alone = pan
        _pan = true;
        _panStartX = event->position().x();
        _panStartY = event->position().y();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton)
    {
        auto cursor_in_scene = this->mapToScene(QPoint(event->position().x(), event->position().y()));

        // Ctrl+Left click = start rotation mode
        if (event->modifiers() & Qt::ControlModifier)
        {
            _robot_dragging = true;
            setCursor(Qt::CrossCursor);
            emit robot_rotate(cursor_in_scene);
            event->accept();
            return;
        }

        // Shift+Left click = instant move robot (legacy behavior)
        if (event->modifiers() & Qt::ShiftModifier)
        {
            emit robot_moved(cursor_in_scene);
            event->accept();
            return;
        }

        // Left click alone = start drag to move robot
        _robot_dragging = true;
        setCursor(Qt::ClosedHandCursor);
        emit robot_drag_start(cursor_in_scene);
        event->accept();
        return;
    }
    if (event->button() == Qt::MiddleButton)
    {
        auto cursor_in_scene = this->mapToScene(QPoint(event->position().x(), event->position().y()));
        emit right_click(cursor_in_scene);
        event->accept();
        return;
    }
    QGraphicsView::mousePressEvent(event);
}
void AbstractGraphicViewer::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        _pan = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
    }
    if (event->button() == Qt::LeftButton)
    {
        if (_robot_dragging)
        {
            _robot_dragging = false;
            setCursor(Qt::ArrowCursor);
            auto cursor_in_scene = this->mapToScene(event->position().toPoint());
            emit robot_drag_end(cursor_in_scene);
            event->accept();
            return;
        }
    }
    QGraphicsView::mouseReleaseEvent(event);
}

void AbstractGraphicViewer::fitToScene(QRectF rect)
{
    // Reset any previous transformations except the Y flip
    resetTransform();
    scale(1, -1);  // Keep Y-axis flipped

    // Fit the given rectangle in the view with some margin
    QRectF paddedRect = rect.adjusted(-rect.width() * 0.1, -rect.height() * 0.1,
                                       rect.width() * 0.1, rect.height() * 0.1);
    fitInView(paddedRect, Qt::KeepAspectRatio);
    centerOn(rect.center());
}
