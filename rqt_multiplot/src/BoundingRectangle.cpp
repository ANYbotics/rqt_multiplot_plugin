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

#include "rqt_multiplot/BoundingRectangle.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

BoundingRectangle::BoundingRectangle(const QPointF& minimum, const QPointF&
    maximum) :
  minimum_(minimum),
  maximum_(maximum) {
}

BoundingRectangle::BoundingRectangle(const QRectF& rectangle) :
  minimum_(rectangle.left(), rectangle.top()),
  maximum_(rectangle.right(), rectangle.bottom()) {
}

BoundingRectangle::BoundingRectangle(const BoundingRectangle& src) :
  minimum_(src.minimum_),
  maximum_(src.maximum_) {
}

BoundingRectangle::~BoundingRectangle() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void BoundingRectangle::setMinimum(const QPointF& minimum) {
  minimum_ = minimum;
}

QPointF& BoundingRectangle::getMinimum() {
  return minimum_;
}

const QPointF& BoundingRectangle::getMinimum() const {
  return minimum_;
}

void BoundingRectangle::setMaximum(const QPointF& maximum) {
  maximum_ = maximum;
}

QPointF& BoundingRectangle::getMaximum() {
  return maximum_;
}

const QPointF& BoundingRectangle::getMaximum() const {
  return maximum_;
}

QRectF BoundingRectangle::getRectangle() const {
  return QRectF(minimum_, maximum_).normalized();
}

bool BoundingRectangle::isValid() const {
  return (maximum_.x() >= minimum_.x()) && (maximum_.y() >= minimum_.y()); 
}

bool BoundingRectangle::isEmpty() const {
  return (maximum_.x() <= minimum_.x()) || (maximum_.y() <= minimum_.y()); 
}

bool BoundingRectangle::contains(const QPointF& point) const {
  return (point.x() >= minimum_.x()) && (point.y() >= minimum_.y()) &&
    (point.x() <= maximum_.x()) && (point.y() <= maximum_.y());
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void BoundingRectangle::initialize(const QPointF& point) {
  minimum_ = point;
  maximum_ = point;
}

void BoundingRectangle::clear() {
  minimum_ = QPointF(0.0, 0.0);
  maximum_ = QPointF(-1.0, -1.0);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

bool BoundingRectangle::operator==(const BoundingRectangle& rectangle) const {
  return (minimum_ == rectangle.minimum_) && (maximum_ == rectangle.maximum_);
}

bool BoundingRectangle::operator!=(const BoundingRectangle& rectangle) const {
  return (minimum_ != rectangle.minimum_) || (maximum_ != rectangle.maximum_);
}

BoundingRectangle& BoundingRectangle::operator+=(const QPointF& point) {
  if (maximum_.x() >= minimum_.x()) {
    minimum_.setX(std::min(minimum_.x(), point.x()));
    maximum_.setX(std::max(maximum_.x(), point.x()));
  }
  else {
    minimum_.setX(point.x());
    maximum_.setX(point.x());
  }
        
  if (maximum_.y() >= minimum_.y()) {
    minimum_.setY(std::min(minimum_.y(), point.y()));
    maximum_.setY(std::max(maximum_.y(), point.y()));
  }
  else {
    minimum_.setY(point.y());
    maximum_.setY(point.y());
  }
  
  return *this;
}

BoundingRectangle& BoundingRectangle::operator+=(const BoundingRectangle&
    rectangle) {
  if (rectangle.maximum_.x() >= rectangle.minimum_.x()) {
    if (maximum_.x() >= minimum_.x()) {
      minimum_.setX(std::min(minimum_.x(), rectangle.minimum_.x()));
      maximum_.setX(std::max(maximum_.x(), rectangle.maximum_.x()));
    }
    else {
      minimum_.setX(rectangle.minimum_.x());
      maximum_.setX(rectangle.maximum_.x());
    }
  }
  
  if (rectangle.maximum_.y() >= rectangle.minimum_.y()) {
    if (maximum_.y() >= minimum_.y()) {
      minimum_.setY(std::min(minimum_.y(), rectangle.minimum_.y()));
      maximum_.setY(std::max(maximum_.y(), rectangle.maximum_.y()));
    }
    else {
      minimum_.setY(rectangle.minimum_.y());
      maximum_.setY(rectangle.maximum_.y());
    }
  }
  
  return *this;
}

}
