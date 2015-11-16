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

#include <QMetaType>

#include "rqt_multiplot/MessageSubscriberRegistry.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Static Initializations                                                    */
/*****************************************************************************/

QMap<QString, MessageSubscriber*> MessageSubscriberRegistry::subscribers_;

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageSubscriberRegistry::MessageSubscriberRegistry(QObject* parent) :
  QObject(parent) {
  qRegisterMetaType<Message>("Message");
}

MessageSubscriberRegistry::~MessageSubscriberRegistry() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const ros::NodeHandle& MessageSubscriberRegistry::getNodeHandle() {
  static ros::NodeHandlePtr nodeHandle(new ros::NodeHandle("~"));
  return *nodeHandle;
}

MessageSubscriber* MessageSubscriberRegistry::getSubscriber(const QString&
    topic) {
  QMap<QString, MessageSubscriber*>::iterator it = subscribers_.find(topic);
    
  if (it == subscribers_.end()) {
    it = subscribers_.insert(topic, new MessageSubscriber(this,
      getNodeHandle()));
    
    it.value()->setTopic(topic);
    it.value()->setQueueSize(100);
    
    connect(it.value(), SIGNAL(destroyed(QObject*)), this,
      SLOT(subscriberDestroyed(QObject*)));
  }
  
  return it.value();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool MessageSubscriberRegistry::subscribe(const QString& topic, QObject*
    receiver, const char* method, size_t queueSize, Qt::ConnectionType
    type) {
  MessageSubscriber* subscriber = getSubscriber(topic);
  
  if (subscriber->getQueueSize() < queueSize)
    subscriber->setQueueSize(queueSize);
  
  return receiver->connect(subscriber, SIGNAL(messageReceived(const
    QString&, const Message&)), method, type);
}

bool MessageSubscriberRegistry::unsubscribe(const QString& topic, QObject*
    receiver, const char* method) {
  QMap<QString, MessageSubscriber*>::iterator it = subscribers_.find(topic);
    
  if (it != subscribers_.end())
    return it.value()->disconnect(SIGNAL(messageReceived(const QString&,
      const Message&)), receiver, method);
  else
    return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageSubscriberRegistry::subscriberDestroyed(QObject* object) {
  for (QMap<QString, MessageSubscriber*>::iterator it = subscribers_.begin();
      it != subscribers_.end(); ++it) {
    if (it.value() == object) {
      subscribers_.erase(it);
      break;
    }
  }
}
  
}
