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

#ifndef TIMESERIESPLOTTER_H
#define TIMESERIESPLOTTER_H

#include <QWidget>
#include <QColor>
#include <QString>
#include <QVector>
#include <QElapsedTimer>
#include "qcustomplot.h"


/**
 * @brief TimeSeriesPlotter - A reusable class for plotting time-series data
 * 
 * This class encapsulates QCustomPlot functionality to provide an easy-to-use
 * interface for real-time time-series visualization. It supports multiple graphs,
 * automatic scrolling, and customizable appearance.
 */
class TimeSeriesPlotter
{
    public:
        /**
         * @brief Configuration parameters for the plotter
         */
        struct Config
        {
            QString title = "Time Series";
            QString xAxisLabel = "Time (s)";
            QString yAxisLabel = "Value";
            double timeWindowSeconds = 10.0;  // Visible time window
            double yMin = -10.0;               // Y-axis minimum
            double yMax = 10.0;                // Y-axis maximum
            bool autoScaleY = true;            // Auto-scale Y axis
            bool showLegend = false;            // Show graph legend
            int maxDataPoints = 10000;         // Maximum points per graph (for memory management)
            Config()
            : title("Time Series")
            , xAxisLabel("Time (s)")
            , yAxisLabel("Value")
            , timeWindowSeconds(10.0)
            , yMin(-10.0)
            , yMax(10.0)
            , autoScaleY(true)
            , showLegend(false)
            , maxDataPoints(10000)
            {}
        };

        /**
         * @brief Constructor
         * @param parent Parent QWidget (typically a QFrame from your UI)
         * @param config Configuration parameters
         */
        explicit TimeSeriesPlotter(QWidget *parent, const Config &config = Config());

        /**
         * @brief Destructor
         */
        ~TimeSeriesPlotter();

        /**
         * @brief Add a new graph to the plot
         * @param name Name of the graph (for legend)
         * @param color Color of the graph line
         * @return Graph index (use this to add data later)
         */
        int addGraph(const QString &name, const QColor &color = Qt::blue);

        /**
         * @brief Add a data point to a specific graph
         * @param graphIndex Index of the graph (returned by addGraph)
         * @param value Y-axis value
         * @param timestamp Optional timestamp (if not provided, uses elapsed time)
         */
        void addDataPoint(int graphIndex, double value, double timestamp = -1.0);

        /**
         * @brief Add data points to multiple graphs at once
         * @param values Vector of values, one per graph (in order of graph creation)
         * @param timestamp Optional timestamp (if not provided, uses elapsed time)
         */
        void addDataPoints(const QVector<double> &values, double timestamp = -1.0);

        /**
         * @brief Update the plot (call this after adding data)
         */
        void update();

        /**
         * @brief Clear all data from all graphs
         */
        void clearData();

        /**
         * @brief Clear data from a specific graph
         * @param graphIndex Index of the graph
         */
        void clearGraph(int graphIndex);

        /**
         * @brief Set Y-axis range
         * @param min Minimum value
         * @param max Maximum value
         */
        void setYRange(double min, double max);

        /**
         * @brief Enable/disable Y-axis auto-scaling
         * @param enable True to enable auto-scaling
         */
        void setAutoScaleY(bool enable);

        /**
         * @brief Set the visible time window
         * @param seconds Time window in seconds
         */
        void setTimeWindow(double seconds);

        /**
         * @brief Get the underlying QCustomPlot widget
         * @return Pointer to QCustomPlot (for advanced customization)
         */
        QCustomPlot* getPlot() { return customPlot; }

        /**
         * @brief Reset the time reference (restart from t=0)
         */
        void resetTime();

    private:
        QCustomPlot *customPlot;
        Config config;
        QElapsedTimer timer;
        QVector<QString> graphNames;

        /**
         * @brief Get current timestamp
         * @param customTimestamp Custom timestamp (-1 to use elapsed time)
         * @return Timestamp in seconds
         */
        double getCurrentTime(double customTimestamp);

        /**
         * @brief Remove old data points to manage memory
         * @param graphIndex Index of the graph
         */
        void pruneOldData(int graphIndex);
};

#endif // TIMESERIESPLOTTER_H
