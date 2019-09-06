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

#include <QApplication>

#include <variant_topic_tools/MessageType.h>

#include <rqt_multiplot/MessageEvent.h>

#include "rqt_multiplot/MessageSubscriber.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageSubscriber::MessageSubscriber(QObject* parent, const ros::NodeHandle&
    nodeHandle) :
  QObject(parent),
  nodeHandle_(nodeHandle),
  queueSize_(100) {
}

MessageSubscriber::~MessageSubscriber() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const ros::NodeHandle& MessageSubscriber::getNodeHandle() const {
  return nodeHandle_;
}

const QString& MessageSubscriber::getTopic() const {
  return topic_;
}

void MessageSubscriber::setTopic(const QString& topic) {
  if (topic != topic_) {
    topic_ = topic;
    
    if (subscriber_) {
      unsubscribe();
      subscribe();
    }
  }
}

void MessageSubscriber::setQueueSize(size_t queueSize) {
  if (queueSize != queueSize_) {
    queueSize_ = queueSize;
    
    if (subscriber_) {
      unsubscribe();
      subscribe();
    }
  }
}

size_t MessageSubscriber::getQueueSize() const {
  return queueSize_;
}

size_t MessageSubscriber::getNumPublishers() const {
  return subscriber_.getNumPublishers();
}

bool MessageSubscriber::isValid() const {
  return subscriber_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool MessageSubscriber::event(QEvent* event) {
  if (event->type() == MessageEvent::Type) {
    MessageEvent* messageEvent = static_cast<MessageEvent*>(event);

    emit messageReceived(messageEvent->getTopic(), messageEvent->
      getMessage());
    
    return true;
  }
  
  return QObject::event(event);
}

void MessageSubscriber::subscribe() {
  variant_topic_tools::MessageType type;
  
  subscriber_ = type.subscribe(nodeHandle_, topic_.toStdString(), queueSize_,
    boost::bind(&MessageSubscriber::callback, this, _1, _2));
  
  if (subscriber_)
    emit subscribed(topic_);
}

void MessageSubscriber::unsubscribe() {
  if (subscriber_) {
    subscriber_.shutdown();
    
    QApplication::removePostedEvents(this, MessageEvent::Type);
    
    emit unsubscribed(topic_);
  }
}

void MessageSubscriber::callback(const variant_topic_tools::MessageVariant&
    variant, const ros::Time& receiptTime) {
  Message message;
  
  message.setReceiptTime(receiptTime);
  message.setVariant(variant);

  MessageEvent* messageEvent = new MessageEvent(topic_, message);
  
  QApplication::postEvent(this, messageEvent);
}

#if QT_VERSION >= QT_VERSION_CHECK(5,0,0)
void MessageSubscriber::connectNotify(const QMetaMethod& signal) {
  if (signal == QMetaMethod::fromSignal(&MessageSubscriber::messageReceived) &&
      !subscriber_)
    subscribe();
}

void MessageSubscriber::disconnectNotify(const QMetaMethod& signal) {
  if (!receivers(QMetaObject::normalizedSignature(
      SIGNAL(messageReceived(const QString&, const Message&))))) {
    if (subscriber_)
      unsubscribe();

    emit aboutToBeDestroyed();

    deleteLater();
  }
}
#else
void MessageSubscriber::connectNotify(const char* signal) {
  if ((QByteArray(signal) == QMetaObject::normalizedSignature(
      SIGNAL(messageReceived(const QString&, const Message&)))) &&
      !subscriber_)
    subscribe();
}

void MessageSubscriber::disconnectNotify(const char* signal) {
  if (!receivers(QMetaObject::normalizedSignature(
      SIGNAL(messageReceived(const QString&, const Message&))))) {
    if (subscriber_)
      unsubscribe();
    
    emit aboutToBeDestroyed();
  
    deleteLater();
  }
}
#endif

}
