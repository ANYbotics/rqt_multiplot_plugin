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

#ifndef RQT_MULTIPLOT_CURVE_DATA_CONFIG_H
#define RQT_MULTIPLOT_CURVE_DATA_CONFIG_H

#include <rqt_multiplot/Config.h>

namespace rqt_multiplot {
  class CurveDataConfig :
    public Config {
  Q_OBJECT
  public:
    enum Type {
      Vector,
      List,
      CircularBuffer,
      TimeFrame
    };
    
    CurveDataConfig(QObject* parent = 0, Type type = Vector, size_t
      circularBufferCapacity = 1000, double timeFrameLength = 10.0);
    ~CurveDataConfig();
    
    void setType(Type type);
    Type getType() const;
    void setCircularBufferCapacity(size_t capacity);
    size_t getCircularBufferCapacity() const;
    void setTimeFrameLength(double length);
    double getTimeFrameLength() const;
    
    void save(QSettings& settings) const;
    void load(QSettings& settings);
    void reset();
    
    void write(QDataStream& stream) const;
    void read(QDataStream& stream);
    
    CurveDataConfig& operator=(const CurveDataConfig& src);
    
  signals:
    void typeChanged(int type);
    void circularBufferCapacityChanged(size_t capacity);
    void timeFrameLengthChanged(double length);
    
  private:
    Type type_;
    size_t circularBufferCapacity_;
    double timeFrameLength_;
  };
};

#endif
