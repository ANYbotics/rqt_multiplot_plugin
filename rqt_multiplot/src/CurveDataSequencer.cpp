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

#include "rqt_multiplot/CurveDataSequencer.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveDataSequencer::CurveDataSequencer(QObject* parent) :
  QObject(parent),
  config_(0),
  registry_(new MessageSubscriberRegistry(this)) {
}

CurveDataSequencer::~CurveDataSequencer() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveDataSequencer::setConfig(CurveConfig* config) {
  if (config != config_) {
    bool wasSubscribed = isSubscribed();
    
    if (config_) {
      disconnect(config_->getAxisConfig(CurveConfig::X),
        SIGNAL(changed()), this, SLOT(configAxisConfigChanged()));
      disconnect(config_->getAxisConfig(CurveConfig::Y),
        SIGNAL(changed()), this, SLOT(configAxisConfigChanged()));
      disconnect(config_, SIGNAL(subscriberQueueSizeChanged(size_t)),
        this, SLOT(configSubscriberQueueSizeChanged(size_t)));
      
      unsubscribe();
    }
    
    config_ = config;
    
    if (config) {
      connect(config->getAxisConfig(CurveConfig::X),
        SIGNAL(changed()), this, SLOT(configAxisConfigChanged()));
      connect(config->getAxisConfig(CurveConfig::Y),
        SIGNAL(changed()), this, SLOT(configAxisConfigChanged()));
      connect(config, SIGNAL(subscriberQueueSizeChanged(size_t)),
        this, SLOT(configSubscriberQueueSizeChanged(size_t)));
      
      if (wasSubscribed)
        subscribe();
    }
  }
}

CurveConfig* CurveDataSequencer::getConfig() const {
  return config_;
}

bool CurveDataSequencer::isSubscribed() const {
  if (!subscribers_.isEmpty()) {
    for (size_t index = 0; index < subscribers_.count(); ++index)
      if (!subscribers_[index]->isValid())
        return false;
      
    return true;
  }
    
  return false;
}
    
/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveDataSequencer::subscribe() {
  if (isSubscribed())
    unsubscribe();
  
  if (config_) {
    CurveAxisConfig* xAxisConfig = config_->getAxisConfig(CurveConfig::X);
    CurveAxisConfig* yAxisConfig = config_->getAxisConfig(CurveConfig::Y);
    
    if (xAxisConfig->getTopic() == yAxisConfig->getTopic()) {
      QString topic = xAxisConfig->getTopic();
      size_t queueSize = config_->getSubscriberQueueSize();
      
      if (registry_->subscribe(topic, this, SLOT(subscriberMessageReceived(
          const QString&, const Message&)), queueSize))
        subscribers_.append(registry_->getSubscriber(topic));
      else
        subscribers_.append(0);
    }
    else {
      QString xTopic = xAxisConfig->getTopic();
      QString yTopic = yAxisConfig->getTopic();
      size_t queueSize = config_->getSubscriberQueueSize();
      
      if (registry_->subscribe(xTopic, this, SLOT(subscriberMessageReceived(
          const QString&, const Message&)), queueSize))
        subscribers_.append(registry_->getSubscriber(xTopic));
      else
        subscribers_.append(0);
      
      if (registry_->subscribe(yTopic, this, SLOT(subscriberMessageReceived(
          const QString&, const Message&)), queueSize))
        subscribers_.append(registry_->getSubscriber(yTopic));
      else
        subscribers_.append(0);
      
      timeFields_.resize(2);
      timeValues_.resize(2);
    }
    
    emit subscribed();
  }
}

void CurveDataSequencer::unsubscribe() {
  for (size_t index = 0; index < subscribers_.count(); ++index)
    registry_->unsubscribe(subscribers_[index]->getTopic(), this,
      SLOT(subscriberMessageReceived(const QString&, const Message&)));
    
  subscribers_.clear();
  timeFields_.clear();
  timeValues_.clear();
  
  emit unsubscribed();
}

// void CurveDataSequencer::interpolate() {
//   while ((timeValues_[0].count() > 1) && (timeValues_[1].count() > 1)) {
//     while ((timeValues_[0].count() > 1) &&
//         ((++timeValues_[0].begin())->time_ < timeValues_[1].front().time_))
//       timeValues_[0].removeFirst();
//     
//     while ((timeValues_[1].count() > 1) &&
//         ((++timeValues_[1].begin())->time_ < timeValues_[0].front().time_))
//       timeValues_[1].removeFirst();
//       
//     size_t i = 0;
//     
//     if (timeValues_[0].front().time_ > timeValues_[1].front().time_)
//       i = 1;
//       
//     QPointF point;
//     const TimeValue& first = timeValues_[i].first();
//     const TimeValue& second = *(++timeValues_[i].begin());
//     
//     if (i == 1) {
//       point.setX(timeValues_[!i].front().value_);
//       point.setY(first.value_+(second.value_-first.value_)*
//         (timeValues_[!i].front().time_-first.time_).toSec()/
//         (second.time_-first.time_).toSec());
//     }
//     else {
//       point.setX(first.value_+(second.value_-first.value_)*
//         (timeValues_[!i].front().time_-first.time_).toSec()/
//         (second.time_-first.time_).toSec());
//       point.setY(timeValues_[!i].front().value_);
//     }
//     
//     timeValues_[i].removeFirst();
//     
//     emit pointReceived(point); 
//   }
// }

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveDataSequencer::configAxisConfigChanged() {
  if (isSubscribed()) {
    unsubscribe();
    subscribe();
  }
}

void CurveDataSequencer::configSubscriberQueueSizeChanged(size_t queueSize) {
  if (isSubscribed()) {
    for (size_t index = 0; index < subscribers_.count(); ++index)
      subscribers_[index]->setQueueSize(queueSize);
  }
}

void CurveDataSequencer::subscriberMessageReceived(const QString& topic,
    const Message& message) {
  if (subscribers_.count() == 1) {
    QPointF point;
    
    CurveAxisConfig* xAxisConfig = config_->getAxisConfig(CurveConfig::X);
    CurveAxisConfig* yAxisConfig = config_->getAxisConfig(CurveConfig::Y);
    
    if (xAxisConfig->getFieldType() == CurveAxisConfig::MessageData) {
      variant_topic_tools::BuiltinVariant variant = message.getVariant().
        getMember(xAxisConfig->getField().toStdString());
        
      point.setX(variant.getNumericValue());
    }
    else
      point.setX(message.getReceiptTime().toSec());
    
    if (yAxisConfig->getFieldType() == CurveAxisConfig::MessageData) {
      variant_topic_tools::BuiltinVariant variant = message.getVariant().
        getMember(yAxisConfig->getField().toStdString());
        
      point.setY(variant.getNumericValue());
    }
    else
      point.setY(message.getReceiptTime().toSec());
    
    emit pointReceived(point);
  }
  else {
//     CurveAxisConfig* axisConfig = 0;
//     size_t axisIndex = 0;
//     
//     if (sender() == subscribers_[0]) {
//       axisConfig = config_->getAxisConfig(CurveConfig::X);
//       axisIndex = 0;
//     }
//     else if (sender() == subscribers_[1]) {
//       axisConfig = config_->getAxisConfig(CurveConfig::Y);
//       axisIndex = 1;
//     }
//     
//     if (axisConfig) {
//       TimeValue timeValue;
//     
//       if (timeValues_[axisIndex].empty() &&
//           (axisConfig->getFieldType() == CurveAxisConfig::MessageData)) {
//         QStringList fieldParts = axisConfig->getField().split("/");
//         
//         while (!fieldParts.isEmpty()) {
//           fieldParts.removeLast();
//           
//           QString parentField = fieldParts.join("/");
//           variant_topic_tools::MessageVariant variant;
//           
//           if (!parentField.isEmpty())
//             variant = message.getVariant().getMember(fieldParts.join("/").
//               toStdString());
//           else
//             variant = message.getVariant();
//             
//           variant_topic_tools::MessageDataType type = variant.getType();
//             
//           if (type.hasHeader()) {
//             timeFields_[axisIndex] = parentField+"/header/stamp";
//             break;
//           }
//         }
//       }
// 
//       if (!timeFields_[axisIndex].isEmpty()) {
//         variant_topic_tools::BuiltinVariant variant = message.getVariant().
//           getMember(timeFields_[axisIndex].toStdString());
//           
//         timeValue.time_ = variant.getValue<ros::Time>();
//       }
//       else
//         timeValue.time_ = message.getReceiptTime();
//       
//       if (axisConfig->getFieldType() == CurveAxisConfig::MessageData) {
//         variant_topic_tools::BuiltinVariant variant = message.getVariant().
//           getMember(axisConfig->getField().toStdString());
//           
//         timeValue.value_ = variant.getNumericValue();
//       }
//       else
//         timeValue.value_ = message.getReceiptTime().toSec();
//         
//       if (!timeValues_[axisIndex].isEmpty() &&
//           (timeValue.time_ <= timeValues_[axisIndex].last().time_))
//         timeValues_[axisIndex].clear();
//         
//       timeValues_[axisIndex].append(timeValue);
//     }
//   
//     interpolate();
//       
//       while ((timeValues_[!axisIndex].count() > 1) &&
//           ((++timeValues_[!axisIndex].begin())->time_ < timeValue.time_))
//         timeValues_[!axisIndex].removeFirst();
//       
//       if (timeValues_[!axisIndex].count() > 1) {
//         QPointF point;
//         
//         const TimeValue& first = timeValues_[!axisIndex].first();
//         const TimeValue& second = *(++timeValues_[!axisIndex].begin());
//         
//         if (axisIndex == 1) {
//           point.setX(first.value_+(second.value_-first.value_)*
//             (timeValue.time_-first.time_).toSec()/
//             (second.time_-first.time_).toSec());
//           point.setY(timeValue.value_);
//         }
//         else {
//           point.setX(timeValue.value_);
//           point.setY(first.value_+(second.value_-first.value_)*
//             (timeValue.time_-first.time_).toSec()/
//             (second.time_-first.time_).toSec());
//         }
//         
//         emit pointReceived(point);
//       }      
//     }
  }
}

}
