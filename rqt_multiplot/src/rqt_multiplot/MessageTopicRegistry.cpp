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

#include <QDir>
#include <QMutexLocker>

#include <ros/master.h>

#include "rqt_multiplot/MessageTopicRegistry.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Static Initializations                                                    */
/*****************************************************************************/

MessageTopicRegistry::Impl MessageTopicRegistry::impl_;

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageTopicRegistry::MessageTopicRegistry(QObject* parent) :
  QObject(parent) {
  connect(&impl_, SIGNAL(started()), this, SLOT(threadStarted()));
  connect(&impl_, SIGNAL(finished()), this, SLOT(threadFinished()));
}

MessageTopicRegistry::~MessageTopicRegistry() {
}

MessageTopicRegistry::Impl::Impl(QObject* parent) :
  QThread(parent) {
}

MessageTopicRegistry::Impl::~Impl() {
  terminate();
  wait();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

QMap<QString, QString> MessageTopicRegistry::getTopics() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.topics_;
}

bool MessageTopicRegistry::isUpdating() const {
  return impl_.isRunning();
}

bool MessageTopicRegistry::isEmpty() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.topics_.isEmpty();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageTopicRegistry::update() {
  impl_.start();
}

void MessageTopicRegistry::Impl::run() {
  std::vector<ros::master::TopicInfo> topics;

  mutex_.lock();
  topics_.clear();
  mutex_.unlock();
  
  if (ros::master::getTopics(topics)) {
    for (size_t i = 0; i < topics.size(); ++i) {
      QString topic = QString::fromStdString(topics[i].name);
      QString type = QString::fromStdString(topics[i].datatype);
      
      topics_[topic] = type;
    }
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageTopicRegistry::threadStarted() {
  emit updateStarted();
}
  
void MessageTopicRegistry::threadFinished() {
  emit updateFinished();
}
  
}
