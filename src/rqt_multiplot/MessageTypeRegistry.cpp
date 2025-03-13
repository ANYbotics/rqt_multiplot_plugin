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

MessageTypeRegistry::MessageTypeRegistry(QObject* parent) : QObject(parent) {
  connect(&impl_, SIGNAL(started()), this, SLOT(threadStarted()));
  connect(&impl_, SIGNAL(finished()), this, SLOT(threadFinished()));
}

MessageTypeRegistry::~MessageTypeRegistry() = default;

MessageTypeRegistry::Impl::Impl(QObject* parent) : QThread(parent) {}

MessageTypeRegistry::Impl::~Impl() {
  terminate();
  wait();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

QList<QString> MessageTypeRegistry::getTypes() {
  QMutexLocker lock(&impl_.mutex_);

  return impl_.types_;
}

bool MessageTypeRegistry::isUpdating() {
  return impl_.isRunning();
}

bool MessageTypeRegistry::isEmpty() {
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
    for (const auto& i : packages) {
      QString package = QString::fromStdString(i);
      QDir directory(QString::fromStdString(ros::package::getPath(i)) + "/msg");

      if (directory.exists()) {
        QList<QString> filters;
        filters.append("*.msg");

        QFileInfoList entries = directory.entryInfoList(filters, QDir::Files | QDir::Readable);

        for (auto& entrie : entries) {
          mutex_.lock();
          types_.append(package + "/" + entrie.baseName());
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

}  // namespace rqt_multiplot
