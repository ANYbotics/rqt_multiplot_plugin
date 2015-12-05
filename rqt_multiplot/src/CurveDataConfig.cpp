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

#include "rqt_multiplot/CurveDataConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveDataConfig::CurveDataConfig(QObject* parent, Type type, size_t
    circularBufferCapacity) :
  QObject(parent),
  type_(type),
  circularBufferCapacity_(circularBufferCapacity) {
}

CurveDataConfig::~CurveDataConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveDataConfig::setType(Type type) {
  if (type != type_) {
    type_ = type;
    
    emit typeChanged(type);
    emit changed();
  }
}

CurveDataConfig::Type CurveDataConfig::getType() const {
  return type_;
}

void CurveDataConfig::setCircularBufferCapacity(size_t capacity) {
  if (capacity != circularBufferCapacity_) {
    circularBufferCapacity_ = capacity;
    
    emit circularBufferCapacityChanged(capacity);
    emit changed();
  }
}

size_t CurveDataConfig::getCircularBufferCapacity() const {
  return circularBufferCapacity_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveDataConfig::save(QSettings& settings) const {
  settings.setValue("type", type_);
  settings.setValue("circular_buffer_capacity", QVariant::
    fromValue<qulonglong>(circularBufferCapacity_));
}

void CurveDataConfig::load(QSettings& settings) {
  setType(static_cast<Type>(settings.value("type").toInt()));
  setCircularBufferCapacity(settings.value("circular_buffer_capacity",
    10000).toULongLong());
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveDataConfig& CurveDataConfig::operator=(const CurveDataConfig& src) {
  setType(src.type_);
  setCircularBufferCapacity(src.circularBufferCapacity_);
  
  return *this;
}

}
