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

#ifndef RQT_MULTIPLOT_CURVE_STYLE_H
#define RQT_MULTIPLOT_CURVE_STYLE_H

#include <QObject>
#include <QSettings>

namespace rqt_multiplot {
  class CurveStyle :
    public QObject {
  Q_OBJECT
  public:
    enum Type {
      Lines,
      Sticks,
      Steps,
      Points
    };
    
    CurveStyle(QObject* parent = 0, Type type = Lines, bool
      linesInterpolate = false, Qt::Orientation sticksOrientation = 
      Qt::Vertical, double sticksBaseline = 0.0, bool stepsInvert =
      false, size_t penWidth = 1, Qt::PenStyle penStyle = Qt::SolidLine, 
      bool renderAntialias = false);
    ~CurveStyle();
    
    void setType(Type type);
    Type getType() const;
    
    void setLinesInterpolate(bool interpolate);
    bool areLinesInterpolated() const;
    void setSticksOrientation(Qt::Orientation orientation);
    Qt::Orientation getSticksOrientation() const;
    void setSticksBaseline(double baseline);
    double getSticksBaseline() const;
    void setStepsInvert(bool invert);
    bool areStepsInverted() const;
    
    void setPenWidth(size_t width);
    size_t getPenWidth() const;
    void setPenStyle(Qt::PenStyle style);
    Qt::PenStyle getPenStyle() const;
    void setRenderAntialias(bool antialias);
    bool isRenderAntialiased() const;
    
    void save(QSettings& settings) const;
    void load(QSettings& settings);
    
    CurveStyle& operator=(const CurveStyle& src);
    
  signals:
    void typeChanged(int type);
    void linesInterpolateChanged(bool interpolate);
    void sticksOrientationChanged(int orientation);
    void sticksBaselineChanged(double baseline);
    void stepsInvertChanged(bool invert);
    void penWidthChanged(size_t width);
    void penStyleChanged(int style);
    void renderAntialiasChanged(bool antialias);
    void changed();
    
  private:
    Type type_;
    
    bool linesInterpolate_;
    Qt::Orientation sticksOrientation_;
    double sticksBaseline_;
    bool stepsInvert_;
    
    size_t penWidth_;
    Qt::PenStyle penStyle_;
    bool renderAntialias_;
  };
};

#endif
