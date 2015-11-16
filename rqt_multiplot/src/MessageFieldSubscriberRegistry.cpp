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

#include "rqt_multiplot/MessageFieldSubscriberRegistry.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Static Initializations                                                    */
/*****************************************************************************/

QMap<QString, MessageFieldSubscriber*> MessageFieldSubscriberRegistry::
  subscribers_;

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldSubscriberRegistry::MessageFieldSubscriberRegistry(QObject*
    parent) :
  QObject(parent) {
}

MessageFieldSubscriberRegistry::~MessageFieldSubscriberRegistry() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

MessageFieldSubscriber* MessageFieldSubscriberRegistry::getSubscriber(const
    QString& topic, const QString& field) {
  QMap<QString, MessageFieldSubscriber*>::iterator it = subscribers_.find(
    topic+":"+field);
    
  if (it == subscribers_.end()) {
    it = subscribers_.insert(topic+":"+field, new
      MessageFieldSubscriber(this));
    
    it.value()->setTopic(topic);
    it.value()->setField(field);
    it.value()->setQueueSize(100);
    
    connect(it.value(), SIGNAL(destroyed(QObject*)), this,
      SLOT(subscriberDestroyed(QObject*)));
  }
  
  return it.value();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool MessageFieldSubscriberRegistry::subscribe(const QString& topic, const
    QString& field, QObject* receiver, const char* method, size_t
    queueSize, Qt::ConnectionType type) {
  MessageFieldSubscriber* subscriber = getSubscriber(topic, field);
  
  if (subscriber->getQueueSize() < queueSize)
    subscriber->setQueueSize(queueSize);
  
  return receiver->connect(subscriber, SIGNAL(valueReceived(const
    QString&, const QString&, double)), method, type);
}

bool MessageFieldSubscriberRegistry::unsubscribe(const QString& topic, const
    QString& field, QObject* receiver, const char* method) {
  QMap<QString, MessageSubscriber*>::iterator it = subscribers_.find(
    topic+":"+field);
    
  if (it != subscribers_.end())
    return it.value()->disconnect(SIGNAL(valueReceived(const QString&, const
      QString&, double)), receiver, method);
  else
    return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageFieldSubscriberRegistry::subscriberDestroyed(QObject* object) {
  for (QMap<QString, MessageFieldSubscriber*>::iterator it = subscribers_.
      begin(); it != subscribers_.end(); ++it) {
    if (it.value() == object) {
      subscribers_.erase(it);
      break;
    }
  }
}
  
}
