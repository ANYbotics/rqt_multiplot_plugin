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

#include <QMouseEvent>

#include <qwt/qwt_painter.h>
#include <qwt/qwt_plot_canvas.h>

#include <rqt_multiplot/PlotZoomerMachine.h>

#include "rqt_multiplot/PlotZoomer.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotZoomer::PlotZoomer(QwtPlotCanvas* canvas, bool doReplot) :
  QwtPlotZoomer(canvas, doReplot) {
  if (canvas)
    setStateMachine(new PlotZoomerMachine());
}

PlotZoomer::~PlotZoomer() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotZoomer::drawRubberBand(QPainter* painter) const {
  if (!isActive())
    return;

  if ((stateMachine()->selectionType() == QwtPickerMachine::RectSelection) &&
      (rubberBand() == RectRubberBand)) {
    if (pickedPoints().count() < 2)
      return;

    QPoint p1 = pickedPoints()[0];
    QPoint p2 = pickedPoints()[pickedPoints().count()-1];

    QRect rect = QRect(p1, p2).normalized();
    rect.adjust(0, 0, -1, -1);

    QwtPainter::drawRect(painter, rect);
  }
  else
    QwtPlotZoomer::drawRubberBand(painter);
}

void PlotZoomer::widgetMousePressEvent(QMouseEvent* event) {
  if (mouseMatch(MouseSelect2, event))
    position_ = event->pos();

  QwtPlotZoomer::widgetMousePressEvent(event);
}

void PlotZoomer::widgetMouseReleaseEvent(QMouseEvent* event) {
  if (mouseMatch(MouseSelect2, event)) {
    if (position_ == event->pos())
      zoom(0);
  }
  else
    QwtPlotZoomer::widgetMouseReleaseEvent(event);
}

}
