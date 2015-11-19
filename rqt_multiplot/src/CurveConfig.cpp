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
#include <boost/concept_check.hpp>

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveConfig::CurveConfig(QObject* parent, const QString& title) :
  QObject(parent),
  title_(title),
  color_(new CurveColor(this)) {
  axisConfig_[X] = new CurveAxisConfig(this);
  axisConfig_[Y] = new CurveAxisConfig(this);
    
  connect(axisConfig_[X], SIGNAL(changed()), this, SLOT(axisConfigChanged()));
  connect(axisConfig_[Y], SIGNAL(changed()), this, SLOT(axisConfigChanged()));
  
  connect(color_, SIGNAL(changed()), this, SLOT(colorChanged()));
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

CurveColor* CurveConfig::getColor() const {
  return color_;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveConfig& CurveConfig::operator=(const CurveConfig& src) {
  setTitle(src.title_);  
  *axisConfig_[X] = *src.axisConfig_[X];
  *axisConfig_[Y] = *src.axisConfig_[Y];
  *color_ = *src.color_;
  
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveConfig::axisConfigChanged() {
  emit changed();
}

void CurveConfig::colorChanged() {
  emit changed();
}

}
