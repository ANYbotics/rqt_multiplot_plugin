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

#ifndef RQT_MULTIPLOT_CURVE_DATA_VECTOR_H
#define RQT_MULTIPLOT_CURVE_DATA_VECTOR_H

#include <QVector>

#include <boost/container/flat_set.hpp>

#include <rqt_multiplot/CurveData.h>

namespace rqt_multiplot {
  class CurveDataVector :
    public CurveData {
  public:
    CurveDataVector();
    ~CurveDataVector();

    size_t getNumPoints() const;
    QPointF getPoint(size_t index) const;
    QVector<size_t> getPointsInDistance(double x, double maxDistance)
      const;
    BoundingRectangle getBounds() const;
    
    void appendPoint(const QPointF& point);
    void clearPoints();
  
  private:
    class XCoordinateRef {
    public:
      inline XCoordinateRef(double x = 0.0, size_t index = 0) :
        x_(x),
        index_(index) {
      };

      inline XCoordinateRef(const XCoordinateRef& src) :
        x_(src.x_),
        index_(src.index_) {
      };
      
      inline bool operator==(const XCoordinateRef& reference) const {
        return (x_ == reference.x_);
      };
      
      inline bool operator<(const XCoordinateRef& reference) const {
        return (x_ < reference.x_);
      };
      
      double x_;
      size_t index_;
    };
    
    typedef boost::container::flat_set<XCoordinateRef> XCoordinateRefSet;
    
    QVector<QPointF> points_;
    XCoordinateRefSet x_;
    
    BoundingRectangle bounds_;
  };
};

#endif
