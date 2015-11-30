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

#ifndef RQT_MULTIPLOT_BOUNDING_RECTANGLE_H
#define RQT_MULTIPLOT_BOUNDING_RECTANGLE_H

#include <QPair>
#include <QPointF>
#include <QRectF>

namespace rqt_multiplot {
  class BoundingRectangle {
  public:
    BoundingRectangle(const QPointF& minimum = QPointF(0.0, 0.0),
      const QPointF& maximum = QPointF(-1.0, -1.0));
    BoundingRectangle(const QRectF& rectangle);
    BoundingRectangle(const BoundingRectangle& src);
    ~BoundingRectangle();
    
    void setMinimum(const QPointF& minimum);
    QPointF& getMinimum();
    const QPointF& getMinimum() const;
    void setMaximum(const QPointF& maximum);
    QPointF& getMaximum();
    const QPointF& getMaximum() const;
    QRectF getRectangle() const;
    bool isValid() const;
    bool isEmpty() const;
    bool contains(const QPointF& point) const;
    
    void initialize(const QPointF& point);
    void clear();
    
    bool operator==(const BoundingRectangle& rectangle) const;
    bool operator!=(const BoundingRectangle& rectangle) const;
    
    BoundingRectangle& operator+=(const QPointF& point);
    BoundingRectangle& operator+=(const BoundingRectangle& rectangle);
    
  private:
    QPointF minimum_;
    QPointF maximum_;
  };
};

#endif
