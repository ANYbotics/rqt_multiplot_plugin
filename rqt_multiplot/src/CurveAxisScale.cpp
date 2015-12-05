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

#include "rqt_multiplot/CurveAxisScale.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveAxisScale::CurveAxisScale(QObject* parent, Type type, double
    absoluteMinimum, double absoluteMaximum, double relativeMinimum,
    double relativeMaximum) :
  QObject(parent),
  type_(type),
  absoluteMinimum_(absoluteMinimum),
  absoluteMaximum_(absoluteMaximum),
  relativeMinimum_(relativeMinimum),
  relativeMaximum_(relativeMaximum) {
}

CurveAxisScale::~CurveAxisScale() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveAxisScale::setType(Type type) {
  if (type != type_) {
    type_ = type;
    
    emit typeChanged(type);
    emit changed();
  }
}

CurveAxisScale::Type CurveAxisScale::getType() const {
  return type_;
}

void CurveAxisScale::setAbsoluteMinimum(double minimum) {
  if (minimum != absoluteMinimum_) {
    absoluteMinimum_ = minimum;
    
    emit absoluteMinimumChanged(minimum);
    emit changed();
  }
}

double CurveAxisScale::getAbsoluteMinimum() const {
  return absoluteMinimum_;
}

void CurveAxisScale::setAbsoluteMaximum(double maximum) {
  if (maximum != absoluteMaximum_) {
    absoluteMaximum_ = maximum;
    
    emit absoluteMaximumChanged(maximum);
    emit changed();
  }
}

double CurveAxisScale::getAbsoluteMaximum() const {
  return absoluteMaximum_;
}

void CurveAxisScale::setRelativeMinimum(double minimum) {
  if (minimum != relativeMinimum_) {
    relativeMinimum_ = minimum;
    
    emit relativeMinimumChanged(minimum);
    emit changed();
  }
}

double CurveAxisScale::getRelativeMinimum() const {
  return relativeMinimum_;
}

void CurveAxisScale::setRelativeMaximum(double maximum) {
  if (maximum != relativeMaximum_) {
    relativeMaximum_ = maximum;
    
    emit relativeMaximumChanged(maximum);
    emit changed();
  }
}

double CurveAxisScale::getRelativeMaximum() const {
  return relativeMaximum_;
}

bool CurveAxisScale::isValid() const {
  if (type_ == Absolute)
    return (absoluteMaximum_ != absoluteMinimum_);
  if (type_ == Relative)
    return (relativeMaximum_ != relativeMinimum_);
  else
    return true;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveAxisScale::save(QSettings& settings) const {
  settings.setValue("type", type_);
  settings.setValue("absolute_minimum", absoluteMinimum_);
  settings.setValue("absolute_maximum", absoluteMaximum_);
  settings.setValue("relative_minimum", relativeMinimum_);
  settings.setValue("relative_maximum", relativeMaximum_);
}

void CurveAxisScale::load(QSettings& settings) {
  setType(static_cast<Type>(settings.value("type", Auto).toInt()));  
  setAbsoluteMinimum(settings.value("absolute_minimum", 0.0).toDouble());
  setAbsoluteMaximum(settings.value("absolute_maximum", 1000.0).toDouble());
  setRelativeMinimum(settings.value("relative_minimum", -1000.0).toDouble());
  setRelativeMaximum(settings.value("relative_maximum", 0.0).toDouble());
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveAxisScale& CurveAxisScale::operator=(const CurveAxisScale& src) {
  setType(src.type_);
  setAbsoluteMinimum(src.absoluteMinimum_);
  setAbsoluteMaximum(src.absoluteMaximum_);
  setRelativeMinimum(src.relativeMinimum_);
  setRelativeMaximum(src.relativeMaximum_);
  
  return *this;
}

}
