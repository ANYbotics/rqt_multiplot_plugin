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

#include <QEvent>
#include <QMouseEvent>

#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_scale_div.h>

#include <ros/package.h>

#include "rqt_multiplot/PlotPanner.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotPanner::PlotPanner(QwtPlotCanvas* canvas) :
  QObject(canvas),
  canvas_(canvas),
  panning_(false) {
  cursor_ = QCursor(QPixmap(QString::fromStdString(ros::package::getPath(
    "rqt_multiplot").append("/resource/23x23/move.png"))), 11, 11);

  if (canvas)
    canvas->installEventFilter(this);
}

PlotPanner::~PlotPanner() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool PlotPanner::eventFilter(QObject* object, QEvent* event) {
  if (object == canvas_) {
    if (!panning_ && (event->type() == QEvent::MouseButtonPress)) {
      QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

      if (mouseEvent->button() == Qt::LeftButton) {
        position_ = mouseEvent->pos();

        xMap_ = canvas_->plot()->canvasMap(QwtPlot::xBottom);
        yMap_ = canvas_->plot()->canvasMap(QwtPlot::yLeft);

        #if QWT_VERSION >= 0x060100
          QPointF minimum(
            canvas_->plot()->axisScaleDiv(QwtPlot::xBottom).lowerBound(),
            canvas_->plot()->axisScaleDiv(QwtPlot::yLeft).lowerBound());
          QPointF maximum(
            canvas_->plot()->axisScaleDiv(QwtPlot::xBottom).upperBound(),
            canvas_->plot()->axisScaleDiv(QwtPlot::yLeft).upperBound());
        #else
          QPointF minimum(
            canvas_->plot()->axisScaleDiv(QwtPlot::xBottom)->lowerBound(),
            canvas_->plot()->axisScaleDiv(QwtPlot::yLeft)->lowerBound());
          QPointF maximum(
            canvas_->plot()->axisScaleDiv(QwtPlot::xBottom)->upperBound(),
            canvas_->plot()->axisScaleDiv(QwtPlot::yLeft)->upperBound());
        #endif

        bounds_.setMinimum(minimum);
        bounds_.setMaximum(maximum);

        canvasCursor_ = canvas_->cursor();
        canvas_->setCursor(cursor_);

        panning_ = true;
      }
    }
    else if (panning_ && (event->type() == QEvent::MouseMove)) {
      QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

      double dx = mouseEvent->pos().x()-position_.x();
      double dy = mouseEvent->pos().y()-position_.y();

      QPointF minimum(
        xMap_.invTransform(xMap_.transform(bounds_.getMinimum().x())-dx),
        yMap_.invTransform(yMap_.transform(bounds_.getMinimum().y())-dy));
      QPointF maximum(
        xMap_.invTransform(xMap_.transform(bounds_.getMaximum().x())-dx),
        yMap_.invTransform(yMap_.transform(bounds_.getMaximum().y())-dy));

      bool autoReplot = canvas_->plot()->autoReplot();
      canvas_->plot()->setAutoReplot(false);

      canvas_->plot()->setAxisScale(QwtPlot::xBottom, minimum.x(),
        maximum.x());
      canvas_->plot()->setAxisScale(QwtPlot::yLeft, minimum.y(),
        maximum.y());

      canvas_->plot()->setAutoReplot(autoReplot);
      canvas_->plot()->replot();
    }
    else if (panning_ && (event->type() == QEvent::MouseButtonRelease)) {
      canvas_->setCursor(canvasCursor_);

      panning_ = false;
    }
  }

  return false;
}

}
