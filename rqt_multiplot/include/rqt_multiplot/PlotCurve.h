/******************************************************************************
 * Copyright (C) 2015 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#ifndef RQT_MULTIPLOT_PLOT_CURVE_H
#define RQT_MULTIPLOT_PLOT_CURVE_H

#include <QObject>
#include <QPair>
#include <QPointF>

#include <rqt_multiplot/BoundingRectangle.h>
#include <rqt_multiplot/CurveConfig.h>
#include <rqt_multiplot/MessageFieldSubscriber.h>
#include <rqt_multiplot/MessageFieldSubscriberRegistry.h>

class QwtPlotCurve;

namespace rqt_multiplot {
  class CurveData;
  class PlotWidget;
  
  class PlotCurve :
    public QObject {
  Q_OBJECT
  friend class PlotWidget;
  public:
    PlotCurve(PlotWidget* parent = 0);
    virtual ~PlotCurve();
    
    void setConfig(CurveConfig* config);
    CurveConfig* getConfig() const;
    QPair<double, double> getPreferredAxisScale(CurveConfig::Axis
      axis) const;
    BoundingRectangle getPreferredScale() const;
    
    void run();
    void pause();
    void clear();
  
    void appendPoint(const QPointF& point);
    
    void replot();
    
  signals:
    void preferredScaleChanged(const BoundingRectangle& bounds);
    
  private:
    CurveConfig* config_;
    
    MessageFieldSubscriberRegistry* registry_;
    MessageFieldSubscriber* subscriberX_;
    MessageFieldSubscriber* subscriberY_;
    
    QwtPlotCurve* curve_;
    CurveData* data_;
    
    QPointF nextPoint_;
    bool paused_;
    
    MessageFieldSubscriber* resubscribe(CurveAxisConfig* axisConfig,
      MessageFieldSubscriber* subscriber, const char* method);
    
  private slots:
    void configTitleChanged(const QString& title);
    void configXAxisConfigChanged();
    void configYAxisConfigChanged();
    void configColorCurrentColorChanged(const QColor& color);
    
    void xValueReceived(const QString& topic, const QString& field,
      double value);
    void yValueReceived(const QString& topic, const QString& field,
      double value);
  };
};

#endif
