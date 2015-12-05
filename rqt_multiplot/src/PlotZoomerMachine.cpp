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

#include <qwt/qwt_event_pattern.h>

#include "rqt_multiplot/PlotZoomerMachine.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotZoomerMachine::PlotZoomerMachine() {
}

PlotZoomerMachine::~PlotZoomerMachine() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

QList<QwtPickerMachine::Command> PlotZoomerMachine::transition(const
    QwtEventPattern& pattern, const QEvent* event) {
  QList<QwtPickerMachine::Command> commands;
  
  if (event->type() == QEvent::MouseButtonDblClick) {
    if (pattern.mouseMatch(QwtEventPattern::MouseSelect1,
        static_cast<const QMouseEvent*>(event))) {
      if (state() == 0) {
        commands += Begin;
        commands += Append;
        commands += Append;
        
        setState(2);
      }
    }
  }
  else if (event->type() != QEvent::MouseButtonPress)
    commands = QwtPickerDragRectMachine::transition(pattern, event);

  return commands;
}

}
