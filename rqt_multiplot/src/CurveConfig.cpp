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

#include "rqt_multiplot/CurveConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Static initializations                                                    */
/*****************************************************************************/

const QString CurveConfig::MimeType = "application/rqt-multiplot-curve-config";

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveConfig::CurveConfig(QObject* parent, const QString& title, size_t
    subscriberQueueSize) :
  Config(parent),
  title_(title),
  colorConfig_(new CurveColorConfig(this)),
  styleConfig_(new CurveStyleConfig(this)),
  dataConfig_(new CurveDataConfig(this)),
  subscriberQueueSize_(subscriberQueueSize) {
  axisConfig_[X] = new CurveAxisConfig(this);
  axisConfig_[Y] = new CurveAxisConfig(this);
    
  connect(axisConfig_[X], SIGNAL(changed()), this, SLOT(axisConfigChanged()));
  connect(axisConfig_[Y], SIGNAL(changed()), this, SLOT(axisConfigChanged()));
  
  connect(colorConfig_, SIGNAL(changed()), this, SLOT(colorConfigChanged()));
  connect(styleConfig_, SIGNAL(changed()), this, SLOT(styleConfigChanged()));
  
  connect(dataConfig_, SIGNAL(changed()), this, SLOT(dataConfigChanged()));
}

CurveConfig::~CurveConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveConfig::setTitle(const QString& title) {
  if (title != title_) {
    title_ = title;
    
    emit titleChanged(title);
    emit changed();
  }
}

const QString& CurveConfig::getTitle() const {
  return title_;
}

CurveAxisConfig* CurveConfig::getAxisConfig(Axis axis) const {
  QMap<Axis, CurveAxisConfig*>::const_iterator it = axisConfig_.find(axis);
  
  if (it != axisConfig_.end())
    return it.value();
  else
    return 0;
}

CurveColorConfig* CurveConfig::getColorConfig() const {
  return colorConfig_;
}

CurveStyleConfig* CurveConfig::getStyleConfig() const {
  return styleConfig_;
}

CurveDataConfig* CurveConfig::getDataConfig() const {
  return dataConfig_;
}

void CurveConfig::setSubscriberQueueSize(size_t queueSize) {
  if (queueSize != subscriberQueueSize_) {
    subscriberQueueSize_ = queueSize;
    
    emit subscriberQueueSizeChanged(queueSize);
    emit changed();
  }
}

size_t CurveConfig::getSubscriberQueueSize() const {
  return subscriberQueueSize_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveConfig::save(QSettings& settings) const {
  settings.setValue("title", title_);
  
  settings.beginGroup("axes");
  settings.beginGroup("x_axis");
  axisConfig_[X]->save(settings);
  settings.endGroup();
  settings.beginGroup("y_axis");
  axisConfig_[Y]->save(settings);
  settings.endGroup();
  settings.endGroup();
  
  settings.beginGroup("color");
  colorConfig_->save(settings);
  settings.endGroup();
  
  settings.beginGroup("style");
  styleConfig_->save(settings);
  settings.endGroup();
  
  settings.beginGroup("data");
  dataConfig_->save(settings);
  settings.endGroup();
  
  settings.setValue("subscriber_queue_size", QVariant::
    fromValue<qulonglong>(subscriberQueueSize_));
}

void CurveConfig::load(QSettings& settings) {
  setTitle(settings.value("title", "Untitled Curve").toString());
  
  settings.beginGroup("axes");
  settings.beginGroup("x_axis");
  axisConfig_[X]->load(settings);
  settings.endGroup();
  settings.beginGroup("y_axis");
  axisConfig_[Y]->load(settings);
  settings.endGroup();
  settings.endGroup();
  
  settings.beginGroup("color");
  colorConfig_->load(settings);
  settings.endGroup();
  
  settings.beginGroup("style");
  styleConfig_->load(settings);
  settings.endGroup();
  
  settings.beginGroup("data");
  dataConfig_->load(settings);
  settings.endGroup();
  
  setSubscriberQueueSize(settings.value("subscriber_queue_size", 100).
    toULongLong());
}

void CurveConfig::reset() {
  setTitle("Untitled Curve");
  
  axisConfig_[X]->reset();
  axisConfig_[Y]->reset();
  
  colorConfig_->reset();
  styleConfig_->reset();
  
  dataConfig_->reset();
  
  setSubscriberQueueSize(100);
}

void CurveConfig::write(QDataStream& stream) const {
  stream << title_;
  
  axisConfig_[X]->write(stream);
  axisConfig_[Y]->write(stream);
  
  colorConfig_->write(stream);
  styleConfig_->write(stream);
  
  dataConfig_->write(stream);
  
  stream << (quint64)subscriberQueueSize_;
}

void CurveConfig::read(QDataStream& stream) {
  QString title;
  quint64 subscriberQueueSize;
  
  stream >> title;
  setTitle(title);
  
  axisConfig_[X]->read(stream);
  axisConfig_[Y]->read(stream);
  
  colorConfig_->read(stream);
  styleConfig_->read(stream);
  
  dataConfig_->read(stream);
  
  stream >> subscriberQueueSize;
  setSubscriberQueueSize(subscriberQueueSize);  
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveConfig& CurveConfig::operator=(const CurveConfig& src) {
  setTitle(src.title_);
  
  *axisConfig_[X] = *src.axisConfig_[X];
  *axisConfig_[Y] = *src.axisConfig_[Y];
  
  *colorConfig_ = *src.colorConfig_;
  *styleConfig_ = *src.styleConfig_;
  
  *dataConfig_ = *src.dataConfig_;
  
  setSubscriberQueueSize(src.subscriberQueueSize_);  
  
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveConfig::axisConfigChanged() {
  emit changed();
}

void CurveConfig::colorConfigChanged() {
  emit changed();
}

void CurveConfig::styleConfigChanged() {
  emit changed();
}

void CurveConfig::dataConfigChanged() {
  emit changed();
}

}
