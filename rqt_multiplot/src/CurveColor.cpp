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

#include "rqt_multiplot/CurveColor.h"

Q_DECLARE_METATYPE(rqt_multiplot::CurveColor::Type)

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveColor::CurveColor(QObject* parent, Type type, unsigned char
    autoColorIndex, const QColor& customColor) :
  QObject(parent),
  type_(type),
  autoColorIndex_(autoColorIndex),
  customColor_(customColor) {
}

CurveColor::~CurveColor() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveColor::setType(Type type) {
  if (type != type_) {
    type_ = type;
    
    emit typeChanged(type);
    emit currentColorChanged(getCurrentColor());
    emit changed();
  }
}

CurveColor::Type CurveColor::getType() const {
  return type_;
}

void CurveColor::setAutoColorIndex(unsigned char index) {
  if (index != autoColorIndex_) {
    autoColorIndex_ = index;
    
    emit autoColorIndexChanged(index);
    emit currentColorChanged(getCurrentColor());
    emit changed();
  }
}

unsigned char CurveColor::getAutoColorIndex() const {
  return autoColorIndex_;
}

void CurveColor::setCustomColor(const QColor& color) {
  if (color != customColor_) {
    customColor_ = color;
    
    emit customColorChanged(color);
    if (type_ == Custom)
      emit currentColorChanged(getCurrentColor());
    emit changed();
  }
}

const QColor& CurveColor::getCustomColor() const {
  return customColor_;
}

QColor CurveColor::getCurrentColor() const {
  if (type_ == Auto)
    return ColorOperations::intToRgb(autoColorIndex_);
  else
    return customColor_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveColor::save(QSettings& settings) const {
  settings.setValue("type", QVariant::fromValue<Type>(type_));
  settings.setValue("custom_color", QVariant::fromValue<QColor>(customColor_));
}

void CurveColor::load(QSettings& settings) {
  setType(settings.value("type").value<Type>());
  setCustomColor(settings.value("custom_color").value<QColor>());
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveColor& CurveColor::operator=(const CurveColor& src) {
  setType(src.type_);
  setAutoColorIndex(src.autoColorIndex_);
  setCustomColor(src.customColor_);
  
  return *this;
}

}
