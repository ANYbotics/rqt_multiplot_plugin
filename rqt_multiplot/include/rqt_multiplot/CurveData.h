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

#ifndef RQT_MULTIPLOT_CURVE_DATA_H
#define RQT_MULTIPLOT_CURVE_DATA_H

#include <QPair>
#include <QPointF>
#include <QRectF>
#include <QVector>

#include <qwt/qwt_series_data.h>

#include <rqt_multiplot/BoundingRectangle.h>
#include <rqt_multiplot/CurveConfig.h>

namespace rqt_multiplot {
  class CurveData :
    public QwtSeriesData<QPointF> {
  public:
    CurveData();
    ~CurveData();

    virtual size_t getNumPoints() const = 0;
    double getValue(size_t index, CurveConfig::Axis axis) const;
    virtual QPointF getPoint(size_t index) const = 0;
    virtual QVector<size_t> getPointsInDistance(double x, double
      maxDistance) const;
    QPair<double, double> getAxisBounds(CurveConfig::Axis axis) const;
    virtual BoundingRectangle getBounds() const = 0;
    bool isEmpty() const;
    
    size_t size() const;
    QPointF sample(size_t i) const;
    QRectF boundingRect() const;
    
    virtual void appendPoint(const QPointF& point) = 0;
    void appendPoint(double x, double y);
    virtual void clearPoints() = 0;
    
    void writeFormatted(QStringList& formattedX, QStringList&
      formattedY) const;
  };
};

#endif
