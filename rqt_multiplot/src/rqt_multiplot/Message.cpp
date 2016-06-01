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

#include "rqt_multiplot/Message.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Message::Message() {
}

Message::Message(const Message& src) :
  receiptTime_(src.receiptTime_),
  variant_(src.variant_) {
}

Message::~Message() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Message::setReceiptTime(const ros::Time& receiptTime) {
  receiptTime_ = receiptTime;
}

const ros::Time& Message::getReceiptTime() const {
  return receiptTime_;
}

void Message::setVariant(const variant_topic_tools::MessageVariant& variant) {
  variant_ = variant;
}

const variant_topic_tools::MessageVariant& Message::getVariant() const {
  return variant_;
}
  
bool Message::isEmpty() const {
  return variant_.isEmpty();
}

}
