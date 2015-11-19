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

#ifndef RQT_MULTIPLOT_CURVE_AXIS_RANGE_H
#define RQT_MULTIPLOT_CURVE_AXIS_RANGE_H

#include <QObject>

namespace rqt_multiplot {
  class CurveAxisRange :
    public QObject {
  Q_OBJECT
  public:
    enum Type {
      Auto,
      Window,
      Fixed
    };
    
    CurveAxisRange(QObject* parent = 0, Type type = Auto, double
      fixedMinimum = 0.0, double fixedMaximum = 1000.0, double
      windowSize = 1000.0);
    ~CurveAxisRange();
    
    void setType(Type type);
    Type getType() const;
    void setFixedMinimum(double minimum);
    double getFixedMinimum() const;
    void setFixedMaximum(double maximum);
    double getFixedMaximum() const;
    void setWindowSize(double size);
    double getWindowSize() const;
    bool isEmpty() const;
    
    CurveAxisRange& operator=(const CurveAxisRange& src);
    
  signals:
    void typeChanged(int type);
    void fixedMinimumChanged(double minimum);
    void fixedMaximumChanged(double maxnimum);
    void windowSizeChanged(double size);
    void changed();
    
  private:
    Type type_;
    double fixedMinimum_;
    double fixedMaximum_;
    double windowSize_;
  };
};

#endif
