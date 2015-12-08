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
          const QString&, const Message&)), queueSize)) {
        subscribers_.append(registry_->getSubscriber(topic));
      }      
    }
    
    emit subscribed();
  }
}

void CurveDataSequencer::unsubscribe() {
  for (size_t index = 0; index < subscribers_.count(); ++index)
    registry_->unsubscribe(subscribers_[index]->getTopic(), this,
      SLOT(subscriberMessageReceived(const QString&, const Message&)));
    
  subscribers_.clear();
  
  emit unsubscribed();
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
}

}
