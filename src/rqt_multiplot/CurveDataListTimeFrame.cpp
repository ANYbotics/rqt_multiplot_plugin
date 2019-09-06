/******************************************************************************
 * Copyright (C) 2015 by Ralf Kaestner, Samuel Bachmann                       *
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

#include "rqt_multiplot/CurveDataListTimeFrame.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveDataListTimeFrame::CurveDataListTimeFrame(double length) :
  timeFrameLength_(length) {
}

CurveDataListTimeFrame::~CurveDataListTimeFrame() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t CurveDataListTimeFrame::getNumPoints() const {
  return points_.count();
}

QPointF CurveDataListTimeFrame::getPoint(size_t index) const {
  return points_[index];
}

BoundingRectangle CurveDataListTimeFrame::getBounds() const {
  return bounds_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveDataListTimeFrame::appendPoint(const QPointF& point) {
  points_.append(point);

  double timeCutoff = point.x() - timeFrameLength_;

//  QMutableListIterator<QPointF> iterator(points_);
//  while (iterator.hasNext()) {
//    if (iterator.next().x() < timeCutoff)
//      iterator.remove();
//    else
//      break;
//  }

  QList<QPointF>::iterator it = points_.begin();
  while (it != points_.end()) {
    if ((*it).x() < timeCutoff)
      it = points_.erase(it);
    else
      break;
  }

  auto min_max_x = std::minmax_element(points_.begin(), points_.end(),
                                       [](const QPointF &a, const QPointF &b) {
                                         return a.x() < b.x();
                                       });

  auto min_max_y = std::minmax_element(points_.begin(), points_.end(),
                                    [](const QPointF &a, const QPointF &b) {
                                      return a.y() < b.y();
                                    });

  bounds_.setMinimum(QPointF(min_max_x.first->x(), min_max_y.first->y()));
  bounds_.setMaximum(QPointF(min_max_x.second->x(), min_max_y.second->y()));
}

void CurveDataListTimeFrame::clearPoints() {
  points_.clear();
  bounds_.clear();
}

}
