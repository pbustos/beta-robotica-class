/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "time_series_plotter.h"
#include <QVBoxLayout>

TimeSeriesPlotter::TimeSeriesPlotter(QWidget *parent, const Config &config)
    : config(config)
{
    // Create the QCustomPlot widget
    customPlot = new QCustomPlot(parent);
    
    // Set up layout if parent has one, otherwise create one
    if (parent->layout() == nullptr)
    {
        QVBoxLayout *layout = new QVBoxLayout(parent);
        layout->setContentsMargins(0, 0, 0, 0);
        parent->setLayout(layout);
    }
    parent->layout()->addWidget(customPlot);

    // Configure the plot appearance
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    
    // Set up axes
    customPlot->xAxis->setLabel(config.xAxisLabel);
    customPlot->yAxis->setLabel(config.yAxisLabel);
    
    // Configure time axis ticker
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    customPlot->xAxis->setTicker(timeTicker);
    
    // Set Y-axis range
    if (!config.autoScaleY)
    {
        customPlot->yAxis->setRange(config.yMin, config.yMax);
    }
    
    // Set up the plot title
    if (!config.title.isEmpty())
    {
        customPlot->plotLayout()->insertRow(0);
        QCPTextElement *title = new QCPTextElement(customPlot, config.title, QFont("sans", 12, QFont::Bold));
        customPlot->plotLayout()->addElement(0, 0, title);
    }
    
    // Configure legend
    if (config.showLegend)
    {
        customPlot->legend->setVisible(true);
        customPlot->legend->setFont(QFont("sans", 9));
        customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignRight);
    }
    
    // Make left and bottom axes transfer their ranges to right and top axes
    //    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)),
    //            customPlot->xAxis2, SLOT(setRange(QCPRange)));
    //    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)),
    //            customPlot->yAxis2, SLOT(setRange(QCPRange)));
    ////

    // Set up full axes box
    customPlot->axisRect()->setupFullAxesBox();
    
    // Start the timer
    timer.start();
}

TimeSeriesPlotter::~TimeSeriesPlotter()
{
    // QCustomPlot will be deleted by Qt's parent-child mechanism
}

int TimeSeriesPlotter::addGraph(const QString &name, const QColor &color)
{
    int graphIndex = customPlot->graphCount();
    customPlot->addGraph();
    
    // Set graph appearance
    QPen pen;
    pen.setColor(color);
    pen.setWidth(2);
    customPlot->graph(graphIndex)->setPen(pen);
    
    // Set graph name for legend
    customPlot->graph(graphIndex)->setName(name);
    graphNames.append(name);
    
    return graphIndex;
}

void TimeSeriesPlotter::addDataPoint(int graphIndex, double value, double timestamp)
{
    if (graphIndex < 0 || graphIndex >= customPlot->graphCount())
    {
        qWarning() << "TimeSeriesPlotter::addDataPoint - Invalid graph index:" << graphIndex;
        return;
    }
    
    double time = getCurrentTime(timestamp);
    customPlot->graph(graphIndex)->addData(time, value);
    
    // Prune old data to manage memory
    pruneOldData(graphIndex);
}

void TimeSeriesPlotter::addDataPoints(const QVector<double> &values, double timestamp)
{
    double time = getCurrentTime(timestamp);
    
    for (int i = 0; i < values.size() && i < customPlot->graphCount(); ++i)
    {
        customPlot->graph(i)->addData(time, values[i]);
        pruneOldData(i);
    }
}

void TimeSeriesPlotter::update()
{
    // Get the current time
    double currentTime = timer.elapsed() / 1000.0;
    
    // Set the x-axis range to show the time window
    customPlot->xAxis->setRange(currentTime, config.timeWindowSeconds, Qt::AlignRight);
    
    // Auto-scale Y axis if enabled
    if (config.autoScaleY && customPlot->graphCount() > 0)
    {
        // Rescale to fit all graphs
        for (int i = 0; i < customPlot->graphCount(); ++i)
        {
            if (i == 0)
                customPlot->graph(i)->rescaleValueAxis(false, true);
            else
                customPlot->graph(i)->rescaleValueAxis(true, true);
        }
    }
    
    // Replot
    customPlot->replot();
}

void TimeSeriesPlotter::clearData()
{
    for (int i = 0; i < customPlot->graphCount(); ++i)
    {
        customPlot->graph(i)->data()->clear();
    }
    customPlot->replot();
}

void TimeSeriesPlotter::clearGraph(int graphIndex)
{
    if (graphIndex >= 0 && graphIndex < customPlot->graphCount())
    {
        customPlot->graph(graphIndex)->data()->clear();
        customPlot->replot();
    }
}

void TimeSeriesPlotter::setYRange(double min, double max)
{
    config.yMin = min;
    config.yMax = max;
    config.autoScaleY = false;
    customPlot->yAxis->setRange(min, max);
    customPlot->replot();
}

void TimeSeriesPlotter::setAutoScaleY(bool enable)
{
    config.autoScaleY = enable;
}

void TimeSeriesPlotter::setTimeWindow(double seconds)
{
    config.timeWindowSeconds = seconds;
}

void TimeSeriesPlotter::resetTime()
{
    timer.restart();
    clearData();
}

double TimeSeriesPlotter::getCurrentTime(double customTimestamp)
{
    if (customTimestamp >= 0.0)
        return customTimestamp;
    else
        return timer.elapsed() / 1000.0;  // Convert milliseconds to seconds
}

void TimeSeriesPlotter::pruneOldData(int graphIndex)
{
    if (graphIndex < 0 || graphIndex >= customPlot->graphCount())
        return;
    
    QSharedPointer<QCPGraphDataContainer> data = customPlot->graph(graphIndex)->data();
    
    // Remove old data points if we exceed the maximum
    if (data->size() > config.maxDataPoints)
    {
        int pointsToRemove = data->size() - config.maxDataPoints;
        data->removeBefore(data->at(pointsToRemove)->key);
    }
}
