#ifndef TIME_SERIES_PLOT_H
#define TIME_SERIES_PLOT_H

#include <QPointF>
#include <QString>
#include <QWidget>
#include <vector>

// Minimal live plot of one signal against time, drawn with QPainter. The x axis spans [0, duration] seconds so a whole
// measurement fits; the y axis is symmetric around zero and grows to fit the largest |value| seen.
class TimeSeriesPlot : public QWidget
{
    public:
        explicit TimeSeriesPlot(QWidget* parent = nullptr);

        void set_labels(const QString& title, const QString& unit);
        void set_duration(double seconds);
        void add(double t, double value);       // t: seconds since the start of the measurement
        void clear();

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        QString title_, unit_;
        double duration_ = 180.0;               // s
        double y_max_ = 1.0;                    // current half-range of the y axis
        std::vector<QPointF> data_;             // (t, value)
};

#endif // TIME_SERIES_PLOT_H
