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

#include "rqt_multiplot/CurveDataSequencer.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveDataSequencer::CurveDataSequencer(QObject* parent) :
  QObject(parent),
  config_(0),
  broker_(0) {
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

void CurveDataSequencer::setBroker(MessageBroker* broker) {
  if (broker != broker_) {
    bool wasSubscribed = isSubscribed();
    
    if (broker_)
      unsubscribe();
    
    broker_ = broker;
    
    if (broker && wasSubscribed)
      subscribe();
  }
}

MessageBroker* CurveDataSequencer::getBroker() const {
  return broker_;
}

bool CurveDataSequencer::isSubscribed() const {
  return !subscribedTopics_.isEmpty();
}
    
/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveDataSequencer::subscribe() {
  if (isSubscribed())
    unsubscribe();
  
  if (config_ && broker_) {
    CurveAxisConfig* xAxisConfig = config_->getAxisConfig(CurveConfig::X);
    CurveAxisConfig* yAxisConfig = config_->getAxisConfig(CurveConfig::Y);
    
    if (xAxisConfig->getTopic() == yAxisConfig->getTopic()) {
      QString topic = xAxisConfig->getTopic();
      
      MessageBroker::PropertyMap properties;
      properties[MessageSubscriber::QueueSize] = QVariant::
        fromValue<qulonglong>(config_->getSubscriberQueueSize());
      
      if (broker_->subscribe(topic, this,
          SLOT(subscriberMessageReceived(const QString&,
          const Message&)), properties)) {
        subscribedTopics_[CurveConfig::X] = topic;
        subscribedTopics_[CurveConfig::Y] = topic;
      }
    }
    else {
      QString xTopic = xAxisConfig->getTopic();
      QString yTopic = yAxisConfig->getTopic();
      
      MessageBroker::PropertyMap properties;
      properties[MessageSubscriber::QueueSize] = QVariant::
        fromValue<qulonglong>(config_->getSubscriberQueueSize());
      
      if (broker_->subscribe(xTopic, this,
          SLOT(subscriberXAxisMessageReceived(const QString&,
          const Message&)), properties))
        subscribedTopics_[CurveConfig::X] = xTopic;
      
      if (broker_->subscribe(yTopic, this,
          SLOT(subscriberYAxisMessageReceived(const QString&,
          const Message&)), properties))
        subscribedTopics_[CurveConfig::Y] = yTopic;
    }
  }
  
  if (!subscribedTopics_.isEmpty())
    emit subscribed();
}

void CurveDataSequencer::unsubscribe() {
  if (isSubscribed()) {
    for (QMap<CurveConfig::Axis, QString>::iterator it = subscribedTopics_.
        begin(); it != subscribedTopics_.end(); ++it)
      broker_->unsubscribe(it.value(), this);
      
    subscribedTopics_.clear();
    timeFields_.clear();
    timeValues_.clear();
    
    emit unsubscribed();
  }
}

void CurveDataSequencer::processMessage(const Message& message) {
  if (!config_)
    return;
  
  CurveAxisConfig* xAxisConfig = config_->getAxisConfig(CurveConfig::X);
  CurveAxisConfig* yAxisConfig = config_->getAxisConfig(CurveConfig::Y);
  
  QPointF point;
  
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

void CurveDataSequencer::processMessage(CurveConfig::Axis axis, const
    Message& message) {
  if (!config_)
    return;
  
  CurveAxisConfig* axisConfig = config_->getAxisConfig(axis);
  
  if (axisConfig) {
    if (!timeFields_.contains(axis)) {
      timeFields_[axis] = QString();
      
      if (axisConfig->getFieldType() == CurveAxisConfig::MessageData) {
        QStringList fieldParts = axisConfig->getField().split("/");
        
        while (!fieldParts.isEmpty()) {
          fieldParts.removeLast();
          
          QString parentField = fieldParts.join("/");
          variant_topic_tools::MessageVariant variant;
          
          if (!parentField.isEmpty())
            variant = message.getVariant().getMember(fieldParts.join("/").
              toStdString());
          else
            variant = message.getVariant();
            
          variant_topic_tools::MessageDataType type = variant.getType();
            
          if (type.hasHeader()) {
            timeFields_[axis] = parentField+"/header/stamp";
            break;
          }
        }
      }
    }

    TimeValue timeValue;
  
    if (!timeFields_[axis].isEmpty()) {
      variant_topic_tools::BuiltinVariant variant = message.getVariant().
        getMember(timeFields_[axis].toStdString());
        
      timeValue.time_ = variant.getValue<ros::Time>();
    }
    else
      timeValue.time_ = message.getReceiptTime();
    
    if (axisConfig->getFieldType() == CurveAxisConfig::MessageData) {
      variant_topic_tools::BuiltinVariant variant = message.getVariant().
        getMember(axisConfig->getField().toStdString());
        
      timeValue.value_ = variant.getNumericValue();
    }
    else
      timeValue.value_ = message.getReceiptTime().toSec();
      
    if (timeValues_[axis].isEmpty() ||
        (timeValue.time_ > timeValues_[axis].last().time_))
      timeValues_[axis].append(timeValue);
  }

  interpolate();
}

void CurveDataSequencer::interpolate() {
  TimeValueList& timeValuesX = timeValues_[CurveConfig::X];
  TimeValueList& timeValuesY = timeValues_[CurveConfig::Y];
  
  while ((timeValuesX.count() > 1) && (timeValuesY.count() > 1)) {
    while ((timeValuesX.count() > 1) &&
        ((++timeValuesX.begin())->time_ < timeValuesY.front().time_))
      timeValuesX.removeFirst();
    
    while ((timeValuesY.count() > 1) &&
        ((++timeValuesY.begin())->time_ < timeValuesX.front().time_))
      timeValuesY.removeFirst();
  
    if ((timeValuesY.front().time_ >= timeValuesX.front().time_) &&
        (timeValuesX.count() > 1)) {
      QPointF point;
      
      const TimeValue& firstX = timeValuesX.first();
      const TimeValue& secondX = *(++timeValuesX.begin());
      
      point.setX(firstX.value_+(secondX.value_-firstX.value_)*
        (timeValuesY.front().time_-firstX.time_).toSec()/
        (secondX.time_-firstX.time_).toSec());
      point.setY(timeValuesY.front().value_);
      
      timeValuesY.removeFirst();
  
      emit pointReceived(point); 
    }    
    else if ((timeValuesX.front().time_ >= timeValuesY.front().time_) &&
        (timeValuesY.count() > 1)) {
      QPointF point;
      
      const TimeValue& firstY = timeValuesY.first();
      const TimeValue& secondY = *(++timeValuesY.begin());
      
      point.setX(timeValuesX.front().value_);
      point.setY(firstY.value_+(secondY.value_-firstY.value_)*
        (timeValuesX.front().time_-firstY.time_).toSec()/
        (secondY.time_-firstY.time_).toSec());
      
      timeValuesX.removeFirst();
      
      emit pointReceived(point); 
    }
  }
}

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
    unsubscribe();
    subscribe();
  }
}

void CurveDataSequencer::subscriberMessageReceived(const QString& topic,
    const Message& message) {
  processMessage(message);
}

void CurveDataSequencer::subscriberXAxisMessageReceived(const QString& topic,
    const Message& message) {      
  processMessage(CurveConfig::X, message);
}

void CurveDataSequencer::subscriberYAxisMessageReceived(const QString& topic,
    const Message& message) {
  processMessage(CurveConfig::Y, message);
}

}
