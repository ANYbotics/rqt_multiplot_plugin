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

#include <qwt/qwt_plot_curve.h>

#include <rqt_multiplot/BoundingRectangle.h>
#include <rqt_multiplot/CurveConfig.h>
#include <rqt_multiplot/MessageFieldSubscriber.h>
#include <rqt_multiplot/MessageFieldSubscriberRegistry.h>

namespace rqt_multiplot {
  class CurveData;
  class CurveDataSequencer;

  class PlotCurve :
    public QObject,
    private QwtPlotCurve {
  Q_OBJECT
  friend class PlotWidget;
  public:
    PlotCurve(QObject* parent = 0);
    virtual ~PlotCurve();
    
    void setConfig(CurveConfig* config);
    CurveConfig* getConfig() const;
    CurveData* getData() const;
    CurveDataSequencer* getDataSequencer() const;
    QPair<double, double> getPreferredAxisScale(CurveConfig::Axis
      axis) const;
    BoundingRectangle getPreferredScale() const;
    
    void attach(QwtPlot* plot);
    void detach();
    
    void run();
    void pause();
    void clear();
  
  signals:
    void preferredScaleChanged(const BoundingRectangle& bounds);
    void replotRequested();
    
  private:
    CurveConfig* config_;
    
    CurveData* data_;
    CurveDataSequencer* dataSequencer_;

    bool paused_;
    
  private slots:
    void configTitleChanged(const QString& title);
    void configAxisConfigChanged();
    void configColorCurrentColorChanged(const QColor& color);
    void configStyleChanged();
    void configDataConfigChanged();
    
    void dataSequencerPointReceived(const QPointF& point);
  };
};

#endif
