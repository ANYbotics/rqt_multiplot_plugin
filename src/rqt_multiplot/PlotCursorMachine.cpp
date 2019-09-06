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

#include "rqt_multiplot/PlotCursorMachine.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotCursorMachine::PlotCursorMachine() {
}

PlotCursorMachine::~PlotCursorMachine() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

QList<QwtPickerMachine::Command> PlotCursorMachine::transition(const
    QwtEventPattern& pattern, const QEvent* event) {
  QList<QwtPickerMachine::Command> commands = QwtPickerTrackerMachine::
    transition(pattern, event);

  if (event->type() == QEvent::Resize) {
    if (state() == 1)
      commands += Move;
  }

  return commands;
}

}
