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

#include <rqt_multiplot/ColorOperations.h>

#include "rqt_multiplot/CurveColorConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveColorConfig::CurveColorConfig(QObject* parent, Type type, unsigned char
    autoColorIndex, const QColor& customColor) :
  Config(parent),
  type_(type),
  autoColorIndex_(autoColorIndex),
  customColor_(customColor) {
}

CurveColorConfig::~CurveColorConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveColorConfig::setType(Type type) {
  if (type != type_) {
    type_ = type;
    
    emit typeChanged(type);
    emit currentColorChanged(getCurrentColor());
    emit changed();
  }
}

CurveColorConfig::Type CurveColorConfig::getType() const {
  return type_;
}

void CurveColorConfig::setAutoColorIndex(unsigned char index) {
  if (index != autoColorIndex_) {
    autoColorIndex_ = index;
    
    emit autoColorIndexChanged(index);
    emit currentColorChanged(getCurrentColor());
    emit changed();
  }
}

unsigned char CurveColorConfig::getAutoColorIndex() const {
  return autoColorIndex_;
}

void CurveColorConfig::setCustomColor(const QColor& color) {
  if (color != customColor_) {
    customColor_ = color;
    
    emit customColorChanged(color);
    if (type_ == Custom)
      emit currentColorChanged(getCurrentColor());
    emit changed();
  }
}

const QColor& CurveColorConfig::getCustomColor() const {
  return customColor_;
}

QColor CurveColorConfig::getCurrentColor() const {
  if (type_ == Auto)
    return ColorOperations::intToRgb(autoColorIndex_);
  else
    return customColor_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveColorConfig::save(QSettings& settings) const {
  settings.setValue("type", type_);
  settings.setValue("custom_color", QVariant::fromValue<QColor>(
    customColor_));
}

void CurveColorConfig::load(QSettings& settings) {
  setType(static_cast<Type>(settings.value("type", Auto).toInt()));
  setCustomColor(settings.value("custom_color", QColor(Qt::black)).
    value<QColor>());
}

void CurveColorConfig::reset() {
  setType(Auto);
  setCustomColor(Qt::black);
}

void CurveColorConfig::write(QDataStream& stream) const {
  stream << type_;
  stream << customColor_;
}

void CurveColorConfig::read(QDataStream& stream) {
  int type;
  QColor customColor;
  
  stream >> type;
  setType(static_cast<Type>(type));
  stream >> customColor;
  setCustomColor(customColor);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveColorConfig& CurveColorConfig::operator=(const CurveColorConfig& src) {
  setType(src.type_);
  setAutoColorIndex(src.autoColorIndex_);
  setCustomColor(src.customColor_);
  
  return *this;
}

}
