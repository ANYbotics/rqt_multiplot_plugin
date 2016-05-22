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

#include <ros/package.h>

#include "rqt_multiplot/MessageTypeRegistry.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Static Initializations                                                    */
/*****************************************************************************/

MessageTypeRegistry::Impl MessageTypeRegistry::impl_;

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageTypeRegistry::MessageTypeRegistry(QObject* parent) :
  QObject(parent) {
  connect(&impl_, SIGNAL(started()), this, SLOT(threadStarted()));
  connect(&impl_, SIGNAL(finished()), this, SLOT(threadFinished()));
}

MessageTypeRegistry::~MessageTypeRegistry() {
}

MessageTypeRegistry::Impl::Impl(QObject* parent) :
  QThread(parent) {
}

MessageTypeRegistry::Impl::~Impl() {
  terminate();
  wait();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

QList<QString> MessageTypeRegistry::getTypes() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.types_;
}

bool MessageTypeRegistry::isUpdating() const {
  return impl_.isRunning();
}

bool MessageTypeRegistry::isEmpty() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.types_.isEmpty();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageTypeRegistry::update() {
  impl_.start();
}

void MessageTypeRegistry::wait() {
  impl_.wait();
}

void MessageTypeRegistry::Impl::run() {
  std::vector<std::string> packages;
  
  mutex_.lock();
  types_.clear();
  mutex_.unlock();
  
  if (ros::package::getAll(packages)) {
    for (size_t i = 0; i < packages.size(); ++i) {
      QString package = QString::fromStdString(packages[i]);
      QDir directory(QString::fromStdString(ros::package::
        getPath(packages[i]))+"/msg");

      if (directory.exists()) {
        QList<QString> filters;
        filters.append("*.msg");
        
        QFileInfoList entries = directory.entryInfoList(filters,
          QDir::Files | QDir::Readable);
        
        for (QFileInfoList::iterator it = entries.begin(); it !=
            entries.end(); ++it) {
          mutex_.lock();
          types_.append(package+"/"+it->baseName());
          mutex_.unlock();
        }
      }
    }
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageTypeRegistry::threadStarted() {
  emit updateStarted();
}
  
void MessageTypeRegistry::threadFinished() {
  emit updateFinished();
}
  
}
