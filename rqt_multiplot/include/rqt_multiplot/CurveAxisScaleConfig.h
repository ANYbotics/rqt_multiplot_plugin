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

#ifndef RQT_MULTIPLOT_CURVE_AXIS_SCALE_CONFIG_H
#define RQT_MULTIPLOT_CURVE_AXIS_SCALE_CONFIG_H

#include <rqt_multiplot/Config.h>

namespace rqt_multiplot {
  class CurveAxisScaleConfig :
    public Config {
  Q_OBJECT
  public:
    enum Type {
      Auto,
      Absolute,
      Relative
    };
    
    CurveAxisScaleConfig(QObject* parent = 0, Type type = Auto, double
      absoluteMinimum = 0.0, double absoluteMaximum = 1000.0, double
      relativeMinimum = -1000.0, double relativeMaximum = 0.0);
    ~CurveAxisScaleConfig();
    
    void setType(Type type);
    Type getType() const;
    void setAbsoluteMinimum(double minimum);
    double getAbsoluteMinimum() const;
    void setAbsoluteMaximum(double maximum);
    double getAbsoluteMaximum() const;
    void setRelativeMinimum(double minimum);
    double getRelativeMinimum() const;
    void setRelativeMaximum(double maximum);
    double getRelativeMaximum() const;
    bool isValid() const;
    
    void save(QSettings& settings) const;
    void load(QSettings& settings);
    void reset();
    
    void write(QDataStream& stream) const;
    void read(QDataStream& stream);
    
    CurveAxisScaleConfig& operator=(const CurveAxisScaleConfig& src);
    
  signals:
    void typeChanged(int type);
    void absoluteMinimumChanged(double minimum);
    void absoluteMaximumChanged(double maxnimum);
    void relativeMinimumChanged(double minimum);
    void relativeMaximumChanged(double maxnimum);
    
  private:
    Type type_;
    double absoluteMinimum_;
    double absoluteMaximum_;
    double relativeMinimum_;
    double relativeMaximum_;
  };
};

#endif
