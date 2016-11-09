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
#include <rqt_multiplot/CurveDataListTimeFrame.h>
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
  broker_(0),
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
      disconnect(config_->getColorConfig(), SIGNAL(currentColorChanged(
        const QColor&)), this, SLOT(configColorConfigCurrentColorChanged(
        const QColor&)));
      disconnect(config_->getStyleConfig(), SIGNAL(changed()), this,
        SLOT(configStyleConfigChanged()));
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
      connect(config->getColorConfig(), SIGNAL(currentColorChanged(
        const QColor&)), this, SLOT(configColorConfigCurrentColorChanged(
        const QColor&)));
      connect(config->getStyleConfig(), SIGNAL(changed()), this,
        SLOT(configStyleConfigChanged()));
      connect(config->getDataConfig(), SIGNAL(changed()), this,
        SLOT(configDataConfigChanged()));
      
      configTitleChanged(config->getTitle());
      configAxisConfigChanged();
      configColorConfigCurrentColorChanged(config->getColorConfig()->
        getCurrentColor());
      configStyleConfigChanged();
      configDataConfigChanged();
      
      dataSequencer_->setConfig(config);
    }
  }
}

CurveConfig* PlotCurve::getConfig() const {
  return config_;
}

void PlotCurve::setBroker(MessageBroker* broker) {
  if (broker != broker_) {
    broker_ = broker;
    
    dataSequencer_->setBroker(broker);
  }
}

MessageBroker* PlotCurve::getBroker() const {
  return broker_;
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
    CurveAxisScaleConfig* axisScaleConfig = config_->getAxisConfig(axis)->
      getScaleConfig();
    
    if (axisScaleConfig->getType() == CurveAxisScaleConfig::Absolute) {
      axisBounds.first = axisScaleConfig->getAbsoluteMinimum();
      axisBounds.second = axisScaleConfig->getAbsoluteMaximum();
    }
    else if (axisScaleConfig->getType() == CurveAxisScaleConfig::Relative) {
      if (!data_->isEmpty()) {
        size_t index = data_->getNumPoints()-1;
        
        axisBounds.first = data_->getValue(index, axis)+axisScaleConfig->
          getRelativeMinimum();
        axisBounds.second = data_->getValue(index, axis)+axisScaleConfig->
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
  CurveAxisConfig* xAxisConfig = config_->getAxisConfig(CurveConfig::X);
  CurveAxisConfig* yAxisConfig = config_->getAxisConfig(CurveConfig::Y);

  if (paused_ &&
      !xAxisConfig->getField().isEmpty() &&
      !yAxisConfig->getField().isEmpty()) {
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

void PlotCurve::configColorConfigCurrentColorChanged(const QColor& color) {
  setPen(color);
  
  emit replotRequested();
}

void PlotCurve::configStyleConfigChanged() {
  rqt_multiplot::CurveStyleConfig* styleConfig = config_->getStyleConfig();
  
  if (styleConfig->getType() == rqt_multiplot::CurveStyleConfig::Sticks) {
    setStyle(QwtPlotCurve::Sticks);
    
    setOrientation(styleConfig->getSticksOrientation());
    setBaseline(styleConfig->getSticksBaseline());
  }
  else if (styleConfig->getType() == rqt_multiplot::CurveStyleConfig::
      Steps) {
    setStyle(QwtPlotCurve::Steps);
    
    setCurveAttribute(QwtPlotCurve::Inverted, styleConfig->
      areStepsInverted());    
  }
  else if (styleConfig->getType() == rqt_multiplot::CurveStyleConfig::
      Points) {
    setStyle(QwtPlotCurve::Dots);
  }
  else {
    setStyle(QwtPlotCurve::Lines);
    
    setCurveAttribute(QwtPlotCurve::Fitted, styleConfig->
      areLinesInterpolated());    
  }
  
  QPen pen = QwtPlotCurve::pen();

  pen.setWidth(styleConfig->getPenWidth());
  pen.setStyle(styleConfig->getPenStyle());
  
  setPen(pen);
  
  setRenderHint(QwtPlotItem::RenderAntialiased, styleConfig->
    isRenderAntialiased());
  
  emit replotRequested();
}

void PlotCurve::configDataConfigChanged() {
  CurveDataConfig* config = config_->getDataConfig();

  if (config->getType() == CurveDataConfig::List)
    data_ = new CurveDataList();
  if (config->getType() == CurveDataConfig::CircularBuffer)
    data_ = new CurveDataCircularBuffer(config->getCircularBufferCapacity());
  if (config->getType() == CurveDataConfig::TimeFrame)
    data_ = new CurveDataListTimeFrame(config->getTimeFrameLength());
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
