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

#include "rqt_multiplot/CurveDataList.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveDataList::CurveDataList() {
}

CurveDataList::~CurveDataList() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t CurveDataList::getNumPoints() const {
  return points_.count();
}

QPointF CurveDataList::getPoint(size_t index) const {
  return points_[index];
}

BoundingRectangle CurveDataList::getBounds() const {
  return bounds_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveDataList::appendPoint(const QPointF& point) {
  bounds_ += point;
  
  points_.append(point);  
}

void CurveDataList::clearPoints() {
  points_.clear();
  bounds_.clear();
}

}
