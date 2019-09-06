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

#include <vector>

#include <QApplication>

#include <rosbag/message_instance.h>

#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/Message.h>
#include <variant_topic_tools/MessageDefinition.h>
#include <variant_topic_tools/MessageType.h>
#include <variant_topic_tools/MessageVariant.h>

#include <rqt_multiplot/DataTypeRegistry.h>
#include <rqt_multiplot/MessageEvent.h>

#include "rqt_multiplot/BagQuery.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

BagQuery::BagQuery(QObject* parent) :
  QObject(parent) {
}

BagQuery::~BagQuery() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool BagQuery::event(QEvent* event) {
  if (event->type() == MessageEvent::Type) {
    MessageEvent* messageEvent = static_cast<MessageEvent*>(event);

    emit messageRead(messageEvent->getTopic(), messageEvent->
      getMessage());
    
    return true;
  }
  
  return QObject::event(event);
}

void BagQuery::callback(const rosbag::MessageInstance& instance) {
  Message message;

  if (!dataType_.isValid()) {
    DataTypeRegistry::mutex_.lock();
    
    variant_topic_tools::DataTypeRegistry registry;
    dataType_ = registry.getDataType(instance.getDataType());

    if (!dataType_) {
      variant_topic_tools::MessageType messageType(instance.getDataType(),
        instance.getMD5Sum(), instance.getMessageDefinition());
      variant_topic_tools::MessageDefinition messageDefinition(
        messageType);
      
      dataType_ = messageDefinition.getMessageDataType();
    }
    
    DataTypeRegistry::mutex_.unlock();
    
    serializer_ = dataType_.createSerializer();
  }

  std::vector<uint8_t> data(instance.size());
  ros::serialization::OStream outputStream(data.data(), data.size());
  instance.write(outputStream);

  variant_topic_tools::MessageVariant variant = dataType_.createVariant();
  ros::serialization::IStream inputStream(data.data(), data.size());
  
  serializer_.deserialize(inputStream, variant);
  
  message.setReceiptTime(instance.getTime());
  message.setVariant(variant);
  
  MessageEvent* messageEvent = new MessageEvent(QString::fromStdString(
    instance.getTopic()), message);
  
  QApplication::postEvent(this, messageEvent);
}

#if QT_VERSION >= QT_VERSION_CHECK(5,0,0)
void BagQuery::disconnectNotify(const QMetaMethod& signal) {
  if (!receivers(QMetaObject::normalizedSignature(
      SIGNAL(messageReceived(const QString&, const Message&))))) {
    emit aboutToBeDestroyed();

    deleteLater();
  }
}
#else
void BagQuery::disconnectNotify(const char* signal) {
  if (!receivers(QMetaObject::normalizedSignature(
      SIGNAL(messageReceived(const QString&, const Message&))))) {
    emit aboutToBeDestroyed();
  
    deleteLater();
  }
}
#endif

}
