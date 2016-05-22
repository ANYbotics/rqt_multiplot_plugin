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
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageSubscriberRegistry::MessageSubscriberRegistry(QObject* parent, const
    ros::NodeHandle& nodeHandle) :
  MessageBroker(parent),
  nodeHandle_(nodeHandle) {
}

MessageSubscriberRegistry::~MessageSubscriberRegistry() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const ros::NodeHandle& MessageSubscriberRegistry::getNodeHandle() const {
  return nodeHandle_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool MessageSubscriberRegistry::subscribe(const QString& topic,
    QObject* receiver, const char* method, const PropertyMap& properties,
    Qt::ConnectionType type) {
  QMap<QString, MessageSubscriber*>::iterator it = subscribers_.find(topic);
  
  size_t queueSize = 100;
  if (properties.contains(MessageSubscriber::QueueSize))
    queueSize = properties[MessageSubscriber::QueueSize].toULongLong();
  
  if (it == subscribers_.end()) {
    it = subscribers_.insert(topic, new MessageSubscriber(this,
      getNodeHandle()));
    
    it.value()->setQueueSize(queueSize);    
    it.value()->setTopic(topic);    
    
    connect(it.value(), SIGNAL(aboutToBeDestroyed()), this,
      SLOT(subscriberAboutToBeDestroyed()));
  }
  else if (it.value()->getQueueSize() < queueSize)
    it.value()->setQueueSize(queueSize);
  
  return receiver->connect(it.value(), SIGNAL(messageReceived(const
    QString&, const Message&)), method, type);
}

bool MessageSubscriberRegistry::unsubscribe(const QString& topic, QObject*
    receiver, const char* method) {
  QMap<QString, MessageSubscriber*>::iterator it = subscribers_.find(topic);
    
  if (it != subscribers_.end()) {
    return it.value()->disconnect(SIGNAL(messageReceived(const QString&,
      const Message&)), receiver, method);
  }
  else
    return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageSubscriberRegistry::subscriberAboutToBeDestroyed() {
  for (QMap<QString, MessageSubscriber*>::iterator it = subscribers_.begin();
      it != subscribers_.end(); ++it) {
    if (it.value() == static_cast<MessageSubscriber*>(sender())) {
      subscribers_.erase(it);      
      break;
    }
  }
}
  
}
