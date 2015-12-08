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

#include <rqt_multiplot/CurveDataCircularBuffer.h>
#include <rqt_multiplot/CurveDataList.h>
#include <rqt_multiplot/CurveDataSequencer.h>
#include <rqt_multiplot/CurveDataVector.h>
#include <rqt_multiplot/PlotWidget.h>

#include "rqt_multiplot/PlotCurve.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotCurve::PlotCurve(QObject* parent) :
  QObject(parent),
  config_(0),
  data_(new CurveDataVector()),
  dataSequencer_(new CurveDataSequencer(this)),
  paused_(true) {
  qRegisterMetaType<BoundingRectangle>("BoundingRectangle");
  
  connect(dataSequencer_, SIGNAL(pointReceived(const QPointF&)), 
    this, SLOT(dataSequencerPointReceived(const QPointF&)));
  
  setData(data_);
}

PlotCurve::~PlotCurve() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotCurve::setConfig(CurveConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(changed(const QString&)), this,
        SLOT(configTitleChanged(const QString&)));
      disconnect(config_->getAxisConfig(CurveConfig::X),
        SIGNAL(changed()), this, SLOT(configAxisConfigChanged()));
      disconnect(config_->getAxisConfig(CurveConfig::Y),
        SIGNAL(changed()), this, SLOT(configAxisConfigChanged()));
      disconnect(config_->getColor(), SIGNAL(currentColorChanged(
        const QColor&)), this, SLOT(configColorCurrentColorChanged(
        const QColor&)));
      disconnect(config_->getStyle(), SIGNAL(changed()), this,
        SLOT(configStyleChanged()));
      disconnect(config_->getDataConfig(), SIGNAL(changed()), this,
        SLOT(configDataConfigChanged()));
      
      dataSequencer_->setConfig(0);
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(titleChanged(const QString&)), this,
        SLOT(configTitleChanged(const QString&)));
      connect(config->getAxisConfig(CurveConfig::X),
        SIGNAL(changed()), this, SLOT(configAxisConfigChanged()));
      connect(config->getAxisConfig(CurveConfig::Y),
        SIGNAL(changed()), this, SLOT(configAxisConfigChanged()));
      connect(config->getColor(), SIGNAL(currentColorChanged(
        const QColor&)), this, SLOT(configColorCurrentColorChanged(
        const QColor&)));
      connect(config->getStyle(), SIGNAL(changed()), this,
        SLOT(configStyleChanged()));
      connect(config->getDataConfig(), SIGNAL(changed()), this,
        SLOT(configDataConfigChanged()));
      
      configTitleChanged(config->getTitle());
      configAxisConfigChanged();
      configColorCurrentColorChanged(config->getColor()->getCurrentColor());
      configStyleChanged();
      configDataConfigChanged();
      
      dataSequencer_->setConfig(config);
    }
  }
}

CurveConfig* PlotCurve::getConfig() const {
  return config_;
}

CurveData* PlotCurve::getData() const {
  return data_;
}

CurveDataSequencer* PlotCurve::getDataSequencer() const {
  return dataSequencer_;
}

QPair<double, double> PlotCurve::getPreferredAxisScale(CurveConfig::Axis axis)
    const {
  QPair<double, double> axisBounds(0.0, -1.0);
  
  if (config_) {
    CurveAxisScale* axisScale = config_->getAxisConfig(axis)->getScale();
    
    if (axisScale->getType() == CurveAxisScale::Absolute) {
      axisBounds.first = axisScale->getAbsoluteMinimum();
      axisBounds.second = axisScale->getAbsoluteMaximum();
    }
    else if (axisScale->getType() == CurveAxisScale::Relative) {
      if (!data_->isEmpty()) {
        size_t index = data_->getNumPoints()-1;
        
        axisBounds.first = data_->getValue(index, axis)+axisScale->
          getRelativeMinimum();
        axisBounds.second = data_->getValue(index, axis)+axisScale->
          getRelativeMaximum();
      }
    }
    else
      axisBounds = data_->getAxisBounds(axis);
  }
  
  return axisBounds;
}

BoundingRectangle PlotCurve::getPreferredScale() const {
  QPair<double, double> xAxisBounds = getPreferredAxisScale(CurveConfig::X);
  QPair<double, double> yAxisBounds = getPreferredAxisScale(CurveConfig::Y);

  return BoundingRectangle(QPointF(xAxisBounds.first, yAxisBounds.first),
    QPointF(xAxisBounds.second, yAxisBounds.second));
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotCurve::attach(QwtPlot* plot) {
  QwtPlotCurve::attach(plot);
}

void PlotCurve::detach() {
  QwtPlotCurve::detach();
}

void PlotCurve::run() {
  if (paused_) {
    dataSequencer_->subscribe();
    
    paused_ = false;
  }
}

void PlotCurve::pause() {
  if (!paused_) {
    dataSequencer_->unsubscribe();
    
    paused_ = true;
  }
}

void PlotCurve::clear() {
  data_->clearPoints();
  
  emit replotRequested();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotCurve::configTitleChanged(const QString& title) {
  setTitle(title);
}

void PlotCurve::configAxisConfigChanged() {
  emit preferredScaleChanged(getPreferredScale());
}

void PlotCurve::configColorCurrentColorChanged(const QColor& color) {
  setPen(color);
  
  emit replotRequested();
}

void PlotCurve::configStyleChanged() {
  rqt_multiplot::CurveStyle* style = config_->getStyle();
  
  if (style->getType() == rqt_multiplot::CurveStyle::Sticks) {
    setStyle(QwtPlotCurve::Sticks);
    
    setOrientation(style->getSticksOrientation());
    setBaseline(style->getSticksBaseline());
  }
  else if (style->getType() == rqt_multiplot::CurveStyle::Steps) {
    setStyle(QwtPlotCurve::Steps);
    
    setCurveAttribute(QwtPlotCurve::Inverted, style->areStepsInverted());    
  }
  else if (style->getType() == rqt_multiplot::CurveStyle::Points) {
    setStyle(QwtPlotCurve::Dots);
  }
  else {
    setStyle(QwtPlotCurve::Lines);
    
    setCurveAttribute(QwtPlotCurve::Fitted, style->areLinesInterpolated());    
  }
  
  QPen pen = QwtPlotCurve::pen();

  pen.setWidth(style->getPenWidth());
  pen.setStyle(style->getPenStyle());
  
  setPen(pen);
  
  setRenderHint(QwtPlotItem::RenderAntialiased, style->
    isRenderAntialiased());
  
  emit replotRequested();
}

void PlotCurve::configDataConfigChanged() {
  CurveDataConfig* config = config_->getDataConfig();

  if (config->getType() == CurveDataConfig::List)
    data_ = new CurveDataList();
  if (config->getType() == CurveDataConfig::CircularBuffer)
    data_ = new CurveDataCircularBuffer(config->getCircularBufferCapacity());
  else
    data_ = new CurveDataVector();
  
  setData(data_);
  
  emit replotRequested();
}

void PlotCurve::dataSequencerPointReceived(const QPointF& point) {
  if (!paused_) {
    BoundingRectangle oldBounds = getPreferredScale();
    
    data_->appendPoint(point);
    
    BoundingRectangle bounds = getPreferredScale();
    
    if (bounds != oldBounds)
      emit preferredScaleChanged(bounds);

    emit replotRequested();
  }
}

}
