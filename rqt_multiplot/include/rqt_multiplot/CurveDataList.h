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

#ifndef RQT_MULTIPLOT_CURVE_DATA_LIST_H
#define RQT_MULTIPLOT_CURVE_DATA_LIST_H

#include <QList>

#include <rqt_multiplot/CurveData.h>

namespace rqt_multiplot {
  class CurveDataList :
    public CurveData {
  public:
    CurveDataList();
    ~CurveDataList();

    size_t getNumPoints() const;
    QPointF getPoint(size_t index) const;
    BoundingRectangle getBounds() const;
    
    void appendPoint(const QPointF& point);
    void clearPoints();
    
  private:
    QList<QPointF> points_;
    BoundingRectangle bounds_;
  };
};

#endif
