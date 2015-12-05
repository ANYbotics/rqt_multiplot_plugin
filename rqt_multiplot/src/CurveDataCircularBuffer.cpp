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

#include "rqt_multiplot/CurveDataCircularBuffer.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveDataCircularBuffer::CurveDataCircularBuffer(size_t capacity) :
  points_(capacity) {
  xMin_.reserve(capacity);
  xMax_.reserve(capacity);
  yMin_.reserve(capacity);
  yMax_.reserve(capacity);
}

CurveDataCircularBuffer::~CurveDataCircularBuffer() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t CurveDataCircularBuffer::getCapacity() const {
  return points_.capacity();
}

size_t CurveDataCircularBuffer::getNumPoints() const {
  return points_.size();
}

QPointF CurveDataCircularBuffer::getPoint(size_t index) const {
  const Point& point = points_[index];
  
  return QPointF(point.x_, point.y_);
}

QVector<size_t> CurveDataCircularBuffer::getPointsInDistance(double x,
    double maxDistance) const {
  QVector<size_t> indexes;
      
  XCoordinateRefMinHeap::ordered_iterator it = std::lower_bound(
    xMin_.ordered_begin(), xMin_.ordered_end(), x-maxDistance);
  
  while (it != xMin_.ordered_end()) {
    if (fabs(x-it->x_) <= maxDistance) {
      size_t index = it->index_;
      
      if (points_.array_two().second) {
        index = (index < points_.array_two().second) ?
          index+points_.array_one().second :
          index-points_.array_two().second;
      }
      
      indexes.push_back(index);
      ++it;
    }
    else
      break;
  }
  
  return indexes;
}

BoundingRectangle CurveDataCircularBuffer::getBounds() const {
  if (!points_.empty()) {
    QPointF minimum(xMin_.top().x_, yMin_.top());
    QPointF maximum(xMax_.top(), yMax_.top());
    
    return BoundingRectangle(minimum, maximum);
  }
  
  
  return BoundingRectangle();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveDataCircularBuffer::appendPoint(const QPointF& point) {
  if (points_.full()) {
    const Point& firstPoint = points_[0];
    
    xMin_.erase(firstPoint.xMinHandle_);
    xMax_.erase(firstPoint.xMaxHandle_);
    yMin_.erase(firstPoint.yMinHandle_);
    yMax_.erase(firstPoint.yMaxHandle_);
  }
  
  points_.push_back(point);
  size_t index = points_.size()-1;

  if (points_.array_two().second)
    index = (points_.array_one().first < points_.array_two().first) ?
      &points_.back()-points_.array_one().first :
      &points_.back()-points_.array_two().first;
  
  points_.back().xMinHandle_ = xMin_.push(XCoordinateRef(point.x(), index));
  points_.back().xMaxHandle_ = xMax_.push(point.x());
  points_.back().yMinHandle_ = yMin_.push(point.y());
  points_.back().yMaxHandle_ = yMax_.push(point.y());  
}

void CurveDataCircularBuffer::clearPoints() {
  points_.clear();
  
  xMin_.clear();
  xMax_.clear();
  yMin_.clear();
  yMax_.clear();
}

}
