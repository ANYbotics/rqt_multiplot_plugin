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

#include <limits>

#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>

#include <rqt_multiplot/CurveListData.h>
#include <rqt_multiplot/PlotWidget.h>

#include "rqt_multiplot/PlotCurve.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotCurve::PlotCurve(PlotWidget* parent) :
  QObject(parent),
  config_(0),
  registry_(new MessageFieldSubscriberRegistry(this)),
  subscriberX_(0),
  subscriberY_(0),
  curve_(new QwtPlotCurve()),
  data_(new CurveListData()),
  nextPoint_(std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()),
  paused_(true) {
  qRegisterMetaType<BoundingRectangle>("BoundingRectangle");
  
  curve_->setData(data_);
}

PlotCurve::~PlotCurve() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

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

void PlotCurve::run() {
  paused_ = false;
}

void PlotCurve::pause() {
  paused_ = true;
}

void PlotCurve::clear() {
  data_->clearPoints();
  
  replot();
}

void PlotCurve::appendPoint(const QPointF& point) {
  BoundingRectangle oldBounds = getPreferredScale();
  
  data_->appendPoint(nextPoint_);
  
  BoundingRectangle bounds = getPreferredScale();
  
  if (bounds != oldBounds)
    emit preferredScaleChanged(bounds);
  else
    replot();
}

void PlotCurve::replot() {
  curve_->plot()->replot();
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
        SIGNAL(changed()), this, SLOT(configXAxisConfigChanged()));
      disconnect(config_->getAxisConfig(CurveConfig::Y),
        SIGNAL(changed()), this, SLOT(configYAxisConfigChanged()));
      disconnect(config_->getColor(), SIGNAL(currentColorChanged(
        const QColor&)), this, SLOT(configColorCurrentColorChanged(
        const QColor&)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(titleChanged(const QString&)), this,
        SLOT(configTitleChanged(const QString&)));
      connect(config->getAxisConfig(CurveConfig::X),
        SIGNAL(changed()), this, SLOT(configXAxisConfigChanged()));
      connect(config->getAxisConfig(CurveConfig::Y),
        SIGNAL(changed()), this, SLOT(configYAxisConfigChanged()));
      connect(config->getColor(), SIGNAL(currentColorChanged(
        const QColor&)), this, SLOT(configColorCurrentColorChanged(
        const QColor&)));
      
      configTitleChanged(config->getTitle());
      configXAxisConfigChanged();
      configYAxisConfigChanged();
      configColorCurrentColorChanged(config->getColor()->getCurrentColor());
    }
  }
}

CurveConfig* PlotCurve::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

MessageFieldSubscriber* PlotCurve::resubscribe(CurveAxisConfig* axisConfig,
    MessageFieldSubscriber* subscriber, const char* method) {
  if (!subscriber || (axisConfig->getTopic() != subscriber->getTopic()) ||
      (axisConfig->getField() != subscriber->getField())) {
    if (subscriber) {
      registry_->unsubscribe(subscriber->getTopic(), subscriber->getField(),
        this, method);
      
      subscriber = 0;
    }

    if (!axisConfig->getTopic().isEmpty() && !axisConfig->getField().
        isEmpty()) {
      if (registry_->subscribe(axisConfig->getTopic(), axisConfig->getField(),
          this, method)) {
        subscriber = registry_->getSubscriber(axisConfig->getTopic(),
          axisConfig->getField());
      }
    }
  }
  
  return subscriber;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotCurve::configTitleChanged(const QString& title) {
  curve_->setTitle(title);
}

void PlotCurve::configXAxisConfigChanged() {
  subscriberX_ = resubscribe(config_->getAxisConfig(CurveConfig::X),
    subscriberX_, SLOT(xValueReceived(const QString&, const QString&,
    double)));
  
  emit preferredScaleChanged(getPreferredScale());
}

void PlotCurve::configYAxisConfigChanged() {
  subscriberY_ = resubscribe(config_->getAxisConfig(CurveConfig::Y),
    subscriberY_, SLOT(yValueReceived(const QString&, const QString&,
    double)));
  
  emit preferredScaleChanged(getPreferredScale());
}

void PlotCurve::configColorCurrentColorChanged(const QColor& color) {
  curve_->setPen(color);
  
  replot();
}

void PlotCurve::xValueReceived(const QString& topic, const QString&
    field, double value) {
  if (!paused_) {
    nextPoint_.setX(value);
    
    if (nextPoint_.y() == nextPoint_.y()) {
      appendPoint(nextPoint_);
      
      nextPoint_.setX(std::numeric_limits<double>::quiet_NaN());
      nextPoint_.setY(std::numeric_limits<double>::quiet_NaN());
    }
  }
}

void PlotCurve::yValueReceived(const QString& topic, const QString&
    field, double value) {
  if (!paused_) {
    nextPoint_.setY(value);
    
    if (nextPoint_.x() == nextPoint_.x()) {
      appendPoint(nextPoint_);
      
      nextPoint_.setX(std::numeric_limits<double>::quiet_NaN());
      nextPoint_.setY(std::numeric_limits<double>::quiet_NaN());

    }
  }
}

}
