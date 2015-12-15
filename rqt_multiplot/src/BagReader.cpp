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
#include <QMutexLocker>

#include <rosbag/bag.h>
#include <ros/console.h>
#include <rosbag/view.h>

#include <rqt_multiplot/ProgressChangeEvent.h>

#include "rqt_multiplot/BagReader.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

BagReader::BagReader(QObject* parent) :
  MessageBroker(parent),
  impl_(this) {
  connect(&impl_, SIGNAL(started()), this, SLOT(threadStarted()));
  connect(&impl_, SIGNAL(finished()), this, SLOT(threadFinished()));
}

BagReader::~BagReader() {
  impl_.quit();
  impl_.wait();
}

BagReader::Impl::Impl(QObject* parent) :
  QThread(parent) {
}

BagReader::Impl::~Impl() {
  terminate();
  wait();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

QString BagReader::getFileName() const {
  return impl_.fileName_;
}

QString BagReader::getError() const {
  return impl_.error_;
}

bool BagReader::isReading() const {
  return impl_.isRunning();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void BagReader::read(const QString& fileName) {
  impl_.wait();
  
  impl_.fileName_ = fileName;
  impl_.error_.clear();
   
  impl_.start();
}

void BagReader::wait() {
  impl_.wait();
}

bool BagReader::subscribe(const QString& topic, QObject* receiver, const
    char* method, const PropertyMap& properties, Qt::ConnectionType type) {
  QMutexLocker lock(&impl_.mutex_);
  
  QMap<QString, BagQuery*>::iterator it = impl_.queries_.find(topic);
  
  if (it == impl_.queries_.end()) {
    it = impl_.queries_.insert(topic, new BagQuery(this));
    
    connect(it.value(), SIGNAL(aboutToBeDestroyed()), this,
      SLOT(queryAboutToBeDestroyed()));
  }
  
  return receiver->connect(it.value(), SIGNAL(messageRead(const
    QString&, const Message&)), method, type);
}

bool BagReader::unsubscribe(const QString& topic, QObject* receiver,
    const char* method) {
  QMutexLocker lock(&impl_.mutex_);
  
  QMap<QString, BagQuery*>::iterator it = impl_.queries_.find(topic);
    
  if (it != impl_.queries_.end()) {
    return it.value()->disconnect(SIGNAL(messageRead(const QString&,
      const Message&)), receiver, method);
  }
  else
    return false;
}

bool BagReader::event(QEvent* event) {
  if (event->type() == ProgressChangeEvent::Type) {
    ProgressChangeEvent* progressChangeEvent = static_cast<
      ProgressChangeEvent*>(event);

    emit readingProgressChanged(progressChangeEvent->getProgress());
    
    return true;
  }
  
  return QObject::event(event);
}

void BagReader::Impl::run() {
  if (queries_.isEmpty())
    return;
  
  try {
    rosbag::Bag bag;
    
    bag.open(fileName_.toStdString(), rosbag::bagmode::Read);
    
    std::vector<std::string> queriedTopics;
    queriedTopics.reserve(queries_.count());
    
    for (QMap<QString, BagQuery*>::const_iterator jt = queries_.begin();
        jt != queries_.end(); ++jt)
      queriedTopics.push_back(jt.key().toStdString());
    
    rosbag::View view(bag, rosbag::TopicQuery(queriedTopics));
    
    for (rosbag::View::iterator it = view.begin(); it != view.end();
        ++it) {
      mutex_.lock();
    
      QMap<QString, BagQuery*>::const_iterator jt = queries_.find(
        QString::fromStdString(it->getTopic()));
      
      if (jt != queries_.end())
        jt.value()->callback(*it);

      mutex_.unlock();
      
      double progress = (it->getTime()-view.getBeginTime()).toSec()/
        (view.getEndTime()-view.getBeginTime()).toSec();

      ProgressChangeEvent* progressChangeEvent = new
        ProgressChangeEvent(progress);
        
      QApplication::postEvent(parent(), progressChangeEvent);
    }
  }
  catch (const ros::Exception& exception) {
    error_ = QString::fromStdString(exception.what());
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void BagReader::threadStarted() {
  emit readingStarted();
}
  
void BagReader::threadFinished() {
  if (impl_.error_.isEmpty()) {
    ROS_INFO_STREAM("Read bag from [file://" <<
      impl_.fileName_.toStdString() << "]");
    
    emit readingFinished();
  }
  else {
    ROS_ERROR_STREAM("Failed to read bag from [file://" <<
      impl_.fileName_.toStdString() << "]: " <<
      impl_.error_.toStdString());
  
    emit readingFailed(impl_.error_);
  }
}

void BagReader::queryAboutToBeDestroyed() {
  for (QMap<QString, BagQuery*>::iterator it = impl_.queries_.begin();
      it != impl_.queries_.end(); ++it) {
    if (it.value() == static_cast<BagQuery*>(sender())) {
      impl_.queries_.erase(it);      
      break;
    }
  }
}

}
