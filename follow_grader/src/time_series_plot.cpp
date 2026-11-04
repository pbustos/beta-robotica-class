#include "time_series_plot.h"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

TimeSeriesPlot::TimeSeriesPlot(QWidget* parent) : QWidget(parent)
{
    setMinimumHeight(140);
}

void TimeSeriesPlot::set_labels(const QString& title, const QString& unit)
{
    title_ = title;
    unit_ = unit;
    update();
}

void TimeSeriesPlot::set_duration(double seconds)
{
    duration_ = std::max(1.0, seconds);
    update();
}

void TimeSeriesPlot::add(double t, double value)
{
    if (not std::isfinite(value)) return;
    data_.emplace_back(t, value);
    // grow the range in 1-2-5 steps so the axis labels stay readable
    while (std::abs(value) > y_max_)
    {
        const double e = std::pow(10.0, std::floor(std::log10(y_max_)));
        const double m = y_max_ / e;
        y_max_ = (m < 1.5 ? 2.0 : m < 3.5 ? 5.0 : 10.0) * e;
    }
    update();
}

void TimeSeriesPlot::clear()
{
    data_.clear();
    y_max_ = 1.0;
    update();
}

void TimeSeriesPlot::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), palette().base());

    const QRectF area(55, 20, width() - 65, height() - 40);   // left margin for the y labels, bottom for the x labels
    auto map = [&](double t, double v)
    {
        return QPointF(area.left() + area.width() * t / duration_, area.center().y() - area.height() / 2.0 * v / y_max_);
    };

    // title, grid and labels
    p.setPen(palette().text().color());
    p.drawText(QRectF(area.left(), 0, area.width(), 18), Qt::AlignLeft | Qt::AlignVCenter, title_ + " (" + unit_ + ")");
    p.setPen(QPen(QColor(200, 200, 200), 1, Qt::DotLine));
    for (double t = 0.0; t <= duration_ + 1e-6; t += duration_ <= 60.0 ? 10.0 : 30.0)
    {
        p.drawLine(map(t, -y_max_), map(t, y_max_));
        p.save(); p.setPen(palette().text().color());
        p.drawText(QRectF(map(t, -y_max_).x() - 20, area.bottom() + 2, 40, 16), Qt::AlignCenter, QString::number(t, 'f', 0));
        p.restore();
    }
    for (const double v : {-y_max_, 0.0, y_max_})
    {
        p.setPen(v == 0.0 ? QPen(QColor(120, 120, 120), 1) : QPen(QColor(200, 200, 200), 1, Qt::DotLine));
        p.drawLine(map(0.0, v), map(duration_, v));
        p.setPen(palette().text().color());
        p.drawText(QRectF(0, map(0.0, v).y() - 8, area.left() - 5, 16), Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(v, 'g', 3));
    }

    // signal
    if (data_.size() < 2) return;
    QPainterPath path(map(data_.front().x(), data_.front().y()));
    for (const auto& d : data_) path.lineTo(map(d.x(), d.y()));
    p.setPen(QPen(QColor(30, 100, 200), 1.5));
    p.drawPath(path);
}
