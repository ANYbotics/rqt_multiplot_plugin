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

#include "rqt_multiplot/CurveData.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveData::CurveData() {
}

CurveData::~CurveData() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

double CurveData::getValue(size_t index, CurveConfig::Axis axis) const {
  if (axis == CurveConfig::X)
    return getPoint(index).x();
  else if (axis == CurveConfig::Y)
    return getPoint(index).y();
    
  return std::numeric_limits<double>::quiet_NaN();
}

QVector<size_t> CurveData::getPointsInDistance(double x, double maxDistance)
    const {
  QVector<size_t> indexes;
  
  if (!isEmpty()) {
    for (size_t index = 0; index < getNumPoints(); ++index) {
      double distance = fabs(x-getPoint(index).x());
      
      if (distance <= maxDistance)
        indexes.append(index);
    }
  }
  
  return indexes;
}

QPair<double, double> CurveData::getAxisBounds(CurveConfig::Axis axis) const {
  BoundingRectangle bounds = getBounds();
  
  if (axis == CurveConfig::X)
    return QPair<double, double>(bounds.getMinimum().x(),
      bounds.getMaximum().x());
  else if (axis == CurveConfig::Y)
    return QPair<double, double>(bounds.getMinimum().y(),
      bounds.getMaximum().y());
  
  return QPair<double, double>();
}

bool CurveData::isEmpty() const {
  return !getNumPoints();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

size_t CurveData::size() const {
  return getNumPoints();
}

QPointF CurveData::sample(size_t i) const {
  return getPoint(i);
}

QRectF CurveData::boundingRect() const {
  return getBounds().getRectangle();
}

void CurveData::appendPoint(double x, double y) {
  appendPoint(QPointF(x, y));
}

void CurveData::writeFormatted(QStringList& formattedX, QStringList&
    formattedY) const {
  formattedX.clear();
  formattedY.clear();
  
  for (size_t index = 0; index < getNumPoints(); ++index) {
    QPointF point = getPoint(index);
    
    formattedX.append(QString::number(point.x(), 'g', 20));
    formattedY.append(QString::number(point.y(), 'g', 20));
  }
}

}
