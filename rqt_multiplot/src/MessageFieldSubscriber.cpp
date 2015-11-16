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

#include <rqt_multiplot/MessageSubscriber.h>

#include "rqt_multiplot/MessageFieldSubscriber.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldSubscriber::MessageFieldSubscriber(QObject* parent) :
  QObject(parent),
  subscriberRegistry_(new MessageSubscriberRegistry(this)),
  subscriber_(0) {
}

MessageFieldSubscriber::~MessageFieldSubscriber() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageFieldSubscriber::setTopic(const QString& topic) {
  if (topic != topic_) {
    topic_ = topic;
    
    if (subscriber_) {
      unsubscribe();
      subscribe();
    }
  }
}

const QString& MessageFieldSubscriber::getTopic() const {
  return topic_;
}

void MessageFieldSubscriber::setField(const QString& field) {
  field_ = field;
}

const QString& MessageFieldSubscriber::getField() const {
  return field_;
}

void MessageFieldSubscriber::setQueueSize(size_t queueSize) {
  if (queueSize != queueSize_) {
    queueSize_ = queueSize;
    
    if (subscriber_)
      subscriber_->setQueueSize(queueSize);
  }
}

size_t MessageFieldSubscriber::getQueueSize() const {
  return queueSize_;
}

size_t MessageFieldSubscriber::getNumPublishers() const {
  if (subscriber_)
    subscriber_->getNumPublishers();
  else
    return 0;
}

bool MessageFieldSubscriber::isValid() const {
  if (subscriber_)
    subscriber_->isValid();
  else
    return false;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageFieldSubscriber::subscribe() {
  if (subscriberRegistry_->subscribe(topic_, this, SLOT(messageReceived(
      const QString&, const Message&)), queueSize_))
    subscriber_ = subscriberRegistry_->getSubscriber(topic_);
  
  if (subscriber_->isValid())
    emit subscribed(topic_, field_);
}

void MessageFieldSubscriber::unsubscribe() {
  if (subscriber_ && subscriberRegistry_->unsubscribe(topic_, this,
      SLOT(messageReceived(const QString&, const Message&)))) {
    subscriber_ = 0;
    emit unsubscribed(topic_, field_);
  }
}

void MessageFieldSubscriber::connectNotify(const char* signal) {
  if ((QByteArray(signal) == QMetaObject::normalizedSignature(
      SIGNAL(valueReceived(const QString&, const QString&, double)))) &&
      !subscriber_)
    subscribe();
}

void MessageFieldSubscriber::disconnectNotify(const char* signal) {
  if (!receivers(QMetaObject::normalizedSignature(
      SIGNAL(valueReceived(const QString&, const QString&, double))))) {
    if (subscriber_)
      unsubscribe();
    
    deleteLater();
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageFieldSubscriber::messageReceived(const QString& topic, const
    Message& message) {
  if (!field_.isEmpty()) {
    variant_topic_tools::BuiltinVariant variant = message.getVariant().
      getMember(field_.toStdString());

    emit valueReceived(topic, field_, variant.getNumericValue());
  }
  else {
    emit valueReceived(topic, field_, message.getReceiptTime().toSec());
  }
}

}
