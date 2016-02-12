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
#include <limits>

#include <QEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QResizeEvent>

#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_scale_widget.h>

#include <rqt_multiplot/CurveData.h>
#include <rqt_multiplot/PlotCursorMachine.h>

#include "rqt_multiplot/PlotCursor.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotCursor::PlotCursor(QwtPlotCanvas* canvas) :
  QwtPlotPicker(canvas),
  trackPoints_(false),
  mouseControl_(false) {
  setTrackerMode(QwtPicker::AlwaysOn);
  setStateMachine(new PlotCursorMachine());

  setRubberBand(QwtPicker::CrossRubberBand);
  setRubberBandPen(Qt::DashLine);

  connect(plot()->axisWidget(xAxis()), SIGNAL(scaleDivChanged()),
    this, SLOT(plotXAxisScaleDivChanged()));
  connect(plot()->axisWidget(yAxis()), SIGNAL(scaleDivChanged()),
    this, SLOT(plotYAxisScaleDivChanged()));
}

PlotCursor::~PlotCursor() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotCursor::setActive(bool active, const QPointF& position) {
  if (mouseControl_)
    return;

  if (active && !isActive()) {
    setTrackerMode(QwtPicker::AlwaysOff);

    begin();
    append(transform(position));

    currentPosition_ = position;

    emit currentPositionChanged(position);
  }
  else if (!active && isActive()) {
    remove();
    end();

    setTrackerMode(QwtPicker::AlwaysOn);
  }
}

void PlotCursor::setCurrentPosition(const QPointF& position) {
  if (mouseControl_)
    return;

  if (isActive() && (position != currentPosition_)) {
    currentPosition_ = position;

    blockSignals(true);
    move(transform(position));
    blockSignals(false);
  }
}

const QPointF& PlotCursor::getCurrentPosition() const {
  return currentPosition_;
}

void PlotCursor::setTrackPoints(bool track) {
  if (track != trackPoints_) {
    trackPoints_ = track;

    if (isActive())
      updateDisplay();
  }
}

bool PlotCursor::arePointsTracked() const {
  return trackPoints_;
}

bool PlotCursor::hasMouseControl() const {
  return mouseControl_;
}

QRect PlotCursor::getTextRect(const QPointF& point, const QFont& font) const {
  QwtText text = trackerTextF(point);

  if (text.isEmpty())
    return QRect();

  QSizeF textSize = text.textSize(font);
  QRect textRect(0, 0, qCeil(textSize.width()), qCeil(textSize.height()));
  QPoint position = transform(point);

  int alignment = Qt::AlignTop | Qt::AlignRight;
  int margin = 5;
  int x = position.x();
  int y = position.y();

  if (alignment & Qt::AlignLeft)
    x -= textRect.width() + margin;
  else if (alignment & Qt::AlignRight)
    x += margin;

  if (alignment & Qt::AlignBottom)
    y += margin;
  else if (alignment & Qt::AlignTop)
    y -= textRect.height() + margin;

  textRect.moveTopLeft(QPoint(x, y));

  #if QWT_VERSION >= 0x060100
    int left = qMax(textRect.left(), trackerRect(font).left()+margin);
    int right = qMin(textRect.right(), trackerRect(font).right()-margin);
    int top = qMax(textRect.top(), trackerRect(font).top()+margin);
    int bottom = qMin( textRect.bottom(), trackerRect(font).bottom()-margin);
  #else
    int left = qMax(textRect.left(), pickRect().left()+margin);
    int right = qMin(textRect.right(), pickRect().right()-margin);
    int top = qMax(textRect.top(), pickRect().top()+margin);
    int bottom = qMin( textRect.bottom(), pickRect().bottom()-margin);
  #endif

  textRect.moveBottomRight(QPoint(right, bottom));
  textRect.moveTopLeft(QPoint(left, top));

  return textRect;
}

QwtText PlotCursor::trackerTextF(const QPointF& point) const {
  QwtScaleMap xMap = plot()->canvasMap(xAxis());
  QwtScaleMap yMap = plot()->canvasMap(yAxis());

  double xPrecision = log10(fabs(xMap.invTransform(1.0)-
    xMap.invTransform(0.0)));
  double yPrecision = log10(fabs(yMap.invTransform(1.0)-
    yMap.invTransform(0.0)));

  QString x, y;

  if ((xPrecision < 0.0) && (fabs(point.x()) >= 1.0))
    x.sprintf("%.*f", (int)ceil(fabs(xPrecision)), point.x());
  else
    x.sprintf("%g", point.x());

  if ((yPrecision < 0.0) && (fabs(point.y()) >= 1.0))
    y.sprintf("%.*f", (int)ceil(fabs(yPrecision)), point.y());
  else
    y.sprintf("%g", point.y());

  return QwtText(x+", "+y);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotCursor::drawRubberBand(QPainter* painter) const {
  if (dynamic_cast<QWidget*>(painter->device())) {
    QPen pen = painter->pen();
    QColor penColor = pen.color();

    penColor.setAlphaF(0.3);
    pen.setColor(penColor);

    painter->setPen(pen);
  }

  QwtPlotPicker::drawRubberBand(painter);

  drawTrackedPoints(painter);
}

void PlotCursor::begin() {
  bool active = isActive();

  QwtPlotPicker::begin();

  if (!active && isActive())
    emit activeChanged(true);
}

void PlotCursor::move(const QPoint& point) {
  QPointF newPosition = invTransform(point);

  if (newPosition != currentPosition_) {
    currentPosition_ = newPosition;

    updateDisplay();

    emit currentPositionChanged(newPosition);
  }

  QwtPlotPicker::move(point);
}

bool PlotCursor::end(bool ok) {
  bool active = isActive();

  bool result = QwtPlotPicker::end(ok);

  if (active && !isActive())
    emit activeChanged(false);

  return result;
}

bool PlotCursor::eventFilter(QObject* object, QEvent* event) {
  if (object == plot()->canvas()) {
    if (event->type() == QEvent::Enter)
      mouseControl_ = true;
    else if (event->type() == QEvent::Leave)
      mouseControl_ = false;
    else if (event->type() == QEvent::MouseButtonRelease)
      updateDisplay();
  }

  bool result = QwtPlotPicker::eventFilter(object, event);

  if (isActive() && object == plot()->canvas()) {
    if (event->type() == QEvent::Resize)
      transition(event);
  }

  return result;
}

void PlotCursor::updateDisplay() {
  updateTrackedPoints();

  QwtPlotPicker::updateDisplay();
}

void PlotCursor::updateTrackedPoints() {
  trackedPoints_.clear();

  if (!trackPoints_ || !isActive())
    return;

  QwtScaleMap map = plot()->canvasMap(xAxis());
  double maxDistance = fabs(map.invTransform(1.0)-map.invTransform(0.0));

  for (QwtPlotItemIterator it = plot()->itemList().begin();
      it != plot()->itemList().end(); ++it) {
    if ((*it)->rtti() == QwtPlotItem::Rtti_PlotCurve) {
      QwtPlotCurve* curve = (QwtPlotCurve*)(*it);
      CurveData* data = dynamic_cast<CurveData*>(curve->data());

      if (data) {
        QVector<size_t> indexes = data->getPointsInDistance(
          currentPosition_.x(), maxDistance);

        if (!indexes.isEmpty()) {
          TrackedPoint trackedPoint;
          trackedPoint.color = curve->pen().color();

          double minDistance = std::numeric_limits<double>::max();

          for (size_t index = 0; index < indexes.count(); ++index) {
            QPointF point = data->getPoint(indexes[index]);
            QPointF vector = currentPosition_-point;
            double distance = vector.x()*vector.x()+vector.y()*vector.y();

            if (distance < minDistance) {
              trackedPoint.position = point;
              minDistance = distance;
            }
          }

          trackedPoints_.append(trackedPoint);
        }
      }
    }
  }
}

void PlotCursor::drawTrackedPoints(QPainter* painter) const {
  if (!trackPoints_)
    return;

  for (size_t index = 0; index < trackedPoints_.count(); ++index) {
    QPointF position = trackedPoints_[index].position;
    QPoint point = transform(position);

    if (dynamic_cast<QWidget*>(painter->device()))
      painter->setPen(trackedPoints_[index].color);

    painter->fillRect(point.x()-3, point.y()-3, 6, 6,
      painter->pen().color());

    QRect textRect = getTextRect(position, painter->font());

    if (!textRect.isEmpty()) {
      if (dynamic_cast<QWidget*>(painter->device())) {
        QwtText label = trackerTextF(position);

        if (!label.isEmpty())
          label.draw(painter, textRect);
      }
      else
        painter->fillRect(textRect, painter->pen().color());
    }
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotCursor::plotXAxisScaleDivChanged() {
  if (isActive()) {
    if (mouseControl_) {
      QPointF newPosition = currentPosition_;

      newPosition.setX(plot()->canvasMap(xAxis()).invTransform(
        pickedPoints()[0].x()));

      if (newPosition != currentPosition_) {
        currentPosition_ = newPosition;

        updateDisplay();

        emit currentPositionChanged(newPosition);
      }
    }
    else {
      QPoint newPosition = pickedPoints()[0];

      newPosition.setX(plot()->canvasMap(xAxis()).transform(
        currentPosition_.x()));

      blockSignals(true);
      move(newPosition);
      blockSignals(false);
    }
  }
}

void PlotCursor::plotYAxisScaleDivChanged() {
  if (isActive()) {
    if (mouseControl_) {
      QPointF newPosition = currentPosition_;

      newPosition.setY(plot()->canvasMap(yAxis()).invTransform(
        pickedPoints()[0].y()));

      if (newPosition != currentPosition_) {
        currentPosition_ = newPosition;

        updateDisplay();

        emit currentPositionChanged(newPosition);
      }
    }
    else {
      QPoint newPosition = pickedPoints()[0];

      newPosition.setY(plot()->canvasMap(yAxis()).transform(
        currentPosition_.y()));

      blockSignals(true);
      move(newPosition);
      blockSignals(false);
    }
  }
}

}
