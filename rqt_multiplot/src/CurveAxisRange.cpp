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

#include "rqt_multiplot/CurveAxisRange.h"

Q_DECLARE_METATYPE(rqt_multiplot::CurveAxisRange::Type)

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveAxisRange::CurveAxisRange(QObject* parent, Type type, double
    fixedMinimum, double fixedMaximum, double windowSize) :
  QObject(parent),
  type_(type),
  fixedMinimum_(fixedMinimum),
  fixedMaximum_(fixedMaximum),
  windowSize_(windowSize) {
}

CurveAxisRange::~CurveAxisRange() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveAxisRange::setType(Type type) {
  if (type != type_) {
    type_ = type;
    
    emit typeChanged(type);
    emit changed();
  }
}

CurveAxisRange::Type CurveAxisRange::getType() const {
  return type_;
}

void CurveAxisRange::setFixedMinimum(double minimum) {
  if (minimum != fixedMinimum_) {
    fixedMinimum_ = minimum;
    
    emit fixedMinimumChanged(minimum);
    emit changed();
  }
}

double CurveAxisRange::getFixedMinimum() const {
  return fixedMinimum_;
}

void CurveAxisRange::setFixedMaximum(double maximum) {
  if (maximum != fixedMaximum_) {
    fixedMaximum_ = maximum;
    
    emit fixedMaximumChanged(maximum);
    emit changed();
  }
}

double CurveAxisRange::getFixedMaximum() const {
  return fixedMaximum_;
}

void CurveAxisRange::setWindowSize(double size) {
  if (size != windowSize_) {
    windowSize_ = size;
    
    emit windowSizeChanged(size);
    emit changed();
  }
}

double CurveAxisRange::getWindowSize() const {
  return windowSize_;
}

bool CurveAxisRange::isEmpty() const {
  if (type_ == Fixed)
    return (fixedMaximum_ <= fixedMinimum_);
  else if (type_ == Window)
    return (windowSize_ <= 0.0);
  else
    return false;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveAxisRange::save(QSettings& settings) const {
  settings.setValue("type", QVariant::fromValue<Type>(type_));
  settings.setValue("fixed_minimum", fixedMinimum_);
  settings.setValue("fixed_maximum", fixedMaximum_);
  settings.setValue("window_size", windowSize_);
}

void CurveAxisRange::load(QSettings& settings) {
  setType(settings.value("type").value<Type>());
  setFixedMinimum(settings.value("fixed_minimum").toDouble());
  setFixedMaximum(settings.value("fixed_maximum").toDouble());
  setWindowSize(settings.value("window_size").toDouble());
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveAxisRange& CurveAxisRange::operator=(const CurveAxisRange& src) {
  setType(src.type_);
  setFixedMinimum(src.fixedMinimum_);
  setFixedMaximum(src.fixedMaximum_);
  setWindowSize(src.windowSize_);
  
  return *this;
}

}
