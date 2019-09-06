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

#include "rqt_multiplot/CurveAxisScaleConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveAxisScaleConfig::CurveAxisScaleConfig(QObject* parent, Type type,
    double absoluteMinimum, double absoluteMaximum, double relativeMinimum,
    double relativeMaximum) :
  Config(parent),
  type_(type),
  absoluteMinimum_(absoluteMinimum),
  absoluteMaximum_(absoluteMaximum),
  relativeMinimum_(relativeMinimum),
  relativeMaximum_(relativeMaximum) {
}

CurveAxisScaleConfig::~CurveAxisScaleConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveAxisScaleConfig::setType(Type type) {
  if (type != type_) {
    type_ = type;
    
    emit typeChanged(type);
    emit changed();
  }
}

CurveAxisScaleConfig::Type CurveAxisScaleConfig::getType() const {
  return type_;
}

void CurveAxisScaleConfig::setAbsoluteMinimum(double minimum) {
  if (minimum != absoluteMinimum_) {
    absoluteMinimum_ = minimum;
    
    emit absoluteMinimumChanged(minimum);
    emit changed();
  }
}

double CurveAxisScaleConfig::getAbsoluteMinimum() const {
  return absoluteMinimum_;
}

void CurveAxisScaleConfig::setAbsoluteMaximum(double maximum) {
  if (maximum != absoluteMaximum_) {
    absoluteMaximum_ = maximum;
    
    emit absoluteMaximumChanged(maximum);
    emit changed();
  }
}

double CurveAxisScaleConfig::getAbsoluteMaximum() const {
  return absoluteMaximum_;
}

void CurveAxisScaleConfig::setRelativeMinimum(double minimum) {
  if (minimum != relativeMinimum_) {
    relativeMinimum_ = minimum;
    
    emit relativeMinimumChanged(minimum);
    emit changed();
  }
}

double CurveAxisScaleConfig::getRelativeMinimum() const {
  return relativeMinimum_;
}

void CurveAxisScaleConfig::setRelativeMaximum(double maximum) {
  if (maximum != relativeMaximum_) {
    relativeMaximum_ = maximum;
    
    emit relativeMaximumChanged(maximum);
    emit changed();
  }
}

double CurveAxisScaleConfig::getRelativeMaximum() const {
  return relativeMaximum_;
}

bool CurveAxisScaleConfig::isValid() const {
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

void CurveAxisScaleConfig::save(QSettings& settings) const {
  settings.setValue("type", type_);
  
  settings.setValue("absolute_minimum", absoluteMinimum_);
  settings.setValue("absolute_maximum", absoluteMaximum_);
  
  settings.setValue("relative_minimum", relativeMinimum_);
  settings.setValue("relative_maximum", relativeMaximum_);
}

void CurveAxisScaleConfig::load(QSettings& settings) {
  setType(static_cast<Type>(settings.value("type", Auto).toInt()));
  
  setAbsoluteMinimum(settings.value("absolute_minimum", 0.0).toDouble());
  setAbsoluteMaximum(settings.value("absolute_maximum", 1000.0).toDouble());
  
  setRelativeMinimum(settings.value("relative_minimum", -1000.0).toDouble());
  setRelativeMaximum(settings.value("relative_maximum", 0.0).toDouble());
}

void CurveAxisScaleConfig::reset() {
  setType(Auto);
  
  setAbsoluteMinimum(0.0);
  setAbsoluteMaximum(1000.0);
  
  setRelativeMinimum(-1000.0);
  setRelativeMaximum(0.0);
}

void CurveAxisScaleConfig::write(QDataStream& stream) const {
  stream << (int)type_;
  
  stream << absoluteMinimum_;
  stream << absoluteMaximum_;
  
  stream << relativeMinimum_;
  stream << relativeMaximum_;
}

void CurveAxisScaleConfig::read(QDataStream& stream) {
  int type;
  double absoluteMinimum, absoluteMaximum, relativeMinimum, relativeMaximum;
  
  stream >> type;
  setType(static_cast<Type>(type));
  
  stream >> absoluteMinimum;
  setAbsoluteMinimum(absoluteMinimum);
  stream >> absoluteMaximum;
  setAbsoluteMaximum(absoluteMaximum);
  
  stream >> relativeMinimum;
  setRelativeMinimum(relativeMinimum);
  stream >> relativeMaximum;
  setRelativeMaximum(relativeMaximum);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveAxisScaleConfig& CurveAxisScaleConfig::operator=(const
    CurveAxisScaleConfig& src) {
  setType(src.type_);
  
  setAbsoluteMinimum(src.absoluteMinimum_);
  setAbsoluteMaximum(src.absoluteMaximum_);
  
  setRelativeMinimum(src.relativeMinimum_);
  setRelativeMaximum(src.relativeMaximum_);
  
  return *this;
}

}
