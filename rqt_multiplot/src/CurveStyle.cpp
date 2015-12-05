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

#include "rqt_multiplot/CurveStyle.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveStyle::CurveStyle(QObject* parent, Type type, bool linesInterpolate,
    Qt::Orientation sticksOrientation, double sticksBaseline,
    bool stepsInvert, size_t penWidth, Qt::PenStyle penStyle, bool
    renderAntialias) :
  QObject(parent),
  type_(type),
  linesInterpolate_(linesInterpolate),
  sticksOrientation_(sticksOrientation),
  sticksBaseline_(sticksBaseline),
  stepsInvert_(stepsInvert),
  penWidth_(penWidth),
  penStyle_(penStyle),
  renderAntialias_(renderAntialias) {
}

CurveStyle::~CurveStyle() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveStyle::setType(Type type) {
  if (type != type_) {
    type_ = type;
    
    emit typeChanged(type);
    emit changed();
  }
}

CurveStyle::Type CurveStyle::getType() const {
  return type_;
}

void CurveStyle::setLinesInterpolate(bool interpolate) {
  if (interpolate != linesInterpolate_) {
    linesInterpolate_ = interpolate;
    
    emit linesInterpolateChanged(interpolate);
    emit changed();
  }
}

bool CurveStyle::areLinesInterpolated() const {
  return linesInterpolate_;
}

void CurveStyle::setSticksOrientation(Qt::Orientation orientation) {
  if (orientation != sticksOrientation_) {
    sticksOrientation_ = orientation;
    
    emit sticksOrientationChanged(orientation);
    emit changed();
  }
}

Qt::Orientation CurveStyle::getSticksOrientation() const {
  return sticksOrientation_;
}

void CurveStyle::setSticksBaseline(double baseline) {
  if (baseline != sticksBaseline_) {
    sticksBaseline_ = baseline;
    
    emit sticksBaselineChanged(baseline);
    emit changed();
  }
}

double CurveStyle::getSticksBaseline() const {
  return sticksBaseline_;
}

void CurveStyle::setStepsInvert(bool invert) {
  if (invert != stepsInvert_) {
    stepsInvert_ = invert;
    
    emit stepsInvertChanged(invert);
    emit changed();
  }
}

bool CurveStyle::areStepsInverted() const {
  return stepsInvert_;
}

void CurveStyle::setPenWidth(size_t width) {
  if (width != penWidth_) {
    penWidth_ = width;
    
    emit penWidthChanged(width);
    emit changed();
  }
}

size_t CurveStyle::getPenWidth() const {
  return penWidth_;
}

void CurveStyle::setPenStyle(Qt::PenStyle style) {
  if (style != penStyle_) {
    penStyle_ = style;
    
    emit penStyleChanged(style);
    emit changed();
  }
}

Qt::PenStyle CurveStyle::getPenStyle() const {
  return penStyle_;
}

void CurveStyle::setRenderAntialias(bool antialias) {
  if (antialias != renderAntialias_) {
    renderAntialias_ = antialias;
    
    emit renderAntialiasChanged(antialias);
    emit changed();
  }
}

bool CurveStyle::isRenderAntialiased() const {
  return renderAntialias_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveStyle::save(QSettings& settings) const {
  settings.setValue("type", type_);
  
  settings.setValue("lines_interpolate", linesInterpolate_);
  settings.setValue("sticks_orientation", sticksOrientation_);
  settings.setValue("sticks_baseline", sticksBaseline_);
  settings.setValue("steps_invert", stepsInvert_);
  
  settings.setValue("pen_width", QVariant::fromValue<qulonglong>(penWidth_));
  settings.setValue("pen_style", penStyle_);
  settings.setValue("render_antialias", renderAntialias_);
}

void CurveStyle::load(QSettings& settings) {
  setType(static_cast<Type>(settings.value("type", Lines).toInt()));
  
  setLinesInterpolate(settings.value("lines_interpolate", false).toBool());
  setSticksOrientation(static_cast<Qt::Orientation>(settings.value(
    "sticks_orientation", Qt::Vertical).toInt()));
  setSticksBaseline(settings.value("sticks_baseline", 0.0).toDouble());
  setStepsInvert(settings.value("steps_invert", false).toBool());
  
  setPenWidth(settings.value("pen_width", 1).toULongLong());
  setPenStyle(static_cast<Qt::PenStyle>(settings.value(
    "pen_style", Qt::SolidLine).toInt()));
  setRenderAntialias(settings.value("render_antialias", false).toBool());
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveStyle& CurveStyle::operator=(const CurveStyle& src) {
  setType(src.type_);
  
  setLinesInterpolate(src.linesInterpolate_);
  setSticksOrientation(src.sticksOrientation_);
  setSticksBaseline(src.sticksBaseline_);
  setStepsInvert(src.stepsInvert_);
  
  setPenWidth(src.penWidth_);
  setPenStyle(src.penStyle_);
  setRenderAntialias(src.renderAntialias_);
  
  return *this;
}

}
