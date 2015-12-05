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

#include "rqt_multiplot/CurveDataVector.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveDataVector::CurveDataVector() {
}

CurveDataVector::~CurveDataVector() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t CurveDataVector::getNumPoints() const {
  return points_.count();
}

QPointF CurveDataVector::getPoint(size_t index) const {
  return points_[index];
}

QVector<size_t> CurveDataVector::getPointsInDistance(double x, double
    maxDistance) const {
  QVector<size_t> indexes;
      
  XCoordinateRefSet::const_iterator it = x_.lower_bound(x-maxDistance);
  
  while (it != x_.end()) {
    if (fabs(x-it->x_) <= maxDistance) {
      indexes.push_back(it->index_);
      ++it;
    }
    else
      break;
  }
  
  return indexes;
}

BoundingRectangle CurveDataVector::getBounds() const {
  return bounds_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveDataVector::appendPoint(const QPointF& point) {
  bounds_ += point;
  
  if (points_.capacity() < points_.count()+1)
    points_.reserve(points_.capacity() ? 2*points_.capacity() : 1);
    
  points_.append(point);
  
  if (x_.capacity() < x_.size()+1)
    x_.reserve(x_.capacity() ? 2*x_.capacity() : 1);
    
  x_.insert(XCoordinateRef(point.x(), points_.size()-1));
}

void CurveDataVector::clearPoints() {
  points_.clear();
  x_.clear();
  
  bounds_.clear();
}

}
