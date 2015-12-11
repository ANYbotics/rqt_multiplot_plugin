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

#include "rqt_multiplot/MessageEvent.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Static initializations                                                    */
/*****************************************************************************/

const QEvent::Type MessageEvent::Type = static_cast<QEvent::Type>(
  QEvent::registerEventType());

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageEvent::MessageEvent(const QString& topic, const Message& message) :
  QEvent(Type),
  topic_(topic),
  message_(message) {
}

MessageEvent::~MessageEvent() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const QString& MessageEvent::getTopic() const {
  return topic_;
}

const Message& MessageEvent::getMessage() const {
  return message_;
}

}
