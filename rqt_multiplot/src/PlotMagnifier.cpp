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

#include <cmath>

#include <QMouseEvent>

#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_scale_div.h>

#include "rqt_multiplot/PlotMagnifier.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotMagnifier::PlotMagnifier(QwtPlotCanvas* canvas) :
  QwtPlotMagnifier(canvas),
  magnifying_(false) {
}

PlotMagnifier::~PlotMagnifier() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotMagnifier::rescale(double xFactor, double yFactor) {
  double fx = std::fabs(xFactor);
  double fy = std::fabs(yFactor);

  if ((fx == 1.0) && (fy == 1.0))
    return;

  bool doReplot = false;
  bool autoReplot = plot()->autoReplot();

  plot()->setAutoReplot(false);

  #if QWT_VERSION >= 0x060100
    const QwtScaleDiv& xScaleDiv = plot()->axisScaleDiv(QwtPlot::xBottom);
    const QwtScaleDiv& yScaleDiv = plot()->axisScaleDiv(QwtPlot::yLeft);
  #else
    const QwtScaleDiv& xScaleDiv = *plot()->axisScaleDiv(QwtPlot::xBottom);
    const QwtScaleDiv& yScaleDiv = *plot()->axisScaleDiv(QwtPlot::yLeft);
  #endif

  #if QWT_VERSION < 0x060100
  if (xScaleDiv.isValid())
  #endif
  {
    double center = xScaleDiv.lowerBound()+0.5*xScaleDiv.range();
    double width = xScaleDiv.range()*fx;

    plot()->setAxisScale(QwtPlot::xBottom, center-0.5*width,
      center+0.5*width);
    doReplot = true;
  }

  #if QWT_VERSION < 0x060100
  if (yScaleDiv.isValid())
  #endif
  {
    double center = yScaleDiv.lowerBound()+0.5*yScaleDiv.range();
    double width = yScaleDiv.range()*fy;

    plot()->setAxisScale(QwtPlot::yLeft, center-0.5*width,
      center+0.5*width);
    doReplot = true;
  }

  plot()->setAutoReplot(autoReplot);

  if (doReplot)
    plot()->replot();
}

void PlotMagnifier::widgetMousePressEvent(QMouseEvent* event) {
  QwtPlotMagnifier::widgetMousePressEvent(event);

  #if QWT_VERSION >= 0x060100
    Qt::MouseButton button;
    Qt::KeyboardModifiers buttonState;
  #else
    int button, buttonState;
  #endif

  getMouseButton(button, buttonState);

  if (event->button() != button || !parentWidget())
    return;

  if ((event->modifiers() & Qt::KeyboardModifierMask) !=
      (int)(buttonState & Qt::KeyboardModifierMask))
    return;

  magnifying_ = true;
  position_ = event->pos();
}

void PlotMagnifier::widgetMouseMoveEvent(QMouseEvent* event) {
  if (!magnifying_)
    return;

  int dx = event->pos().x()-position_.x();
  int dy = event->pos().y()-position_.y();

  double fx = 1.0;
  double fy = 1.0;

  if (dx != 0) {
    fx = mouseFactor();

    if (dx < 0)
      fx = 1.0/fx;
  }

  if (dy != 0) {
    fy = mouseFactor();

    if (dy < 0)
      fy = 1.0/fy;
  }

  rescale(fx, fy);

  position_ = event->pos();
}

void PlotMagnifier::widgetMouseReleaseEvent(QMouseEvent* event) {
  QwtPlotMagnifier::widgetMouseReleaseEvent(event);

  magnifying_ = false;
}

}
