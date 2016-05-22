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

#include "rqt_multiplot/PackageRegistry.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Static Initializations                                                    */
/*****************************************************************************/

PackageRegistry::Impl PackageRegistry::impl_;

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PackageRegistry::PackageRegistry(QObject* parent) :
  QObject(parent) {
  connect(&impl_, SIGNAL(started()), this, SLOT(threadStarted()));
  connect(&impl_, SIGNAL(finished()), this, SLOT(threadFinished()));
}

PackageRegistry::~PackageRegistry() {
}

PackageRegistry::Impl::Impl(QObject* parent) :
  QThread(parent) {
}

PackageRegistry::Impl::~Impl() {
  terminate();
  wait();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

QMap<QString, QString> PackageRegistry::getPackages() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.packages_;
}

bool PackageRegistry::isUpdating() const {
  return impl_.isRunning();
}

bool PackageRegistry::isEmpty() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.packages_.isEmpty();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PackageRegistry::update() {
  impl_.start();
}

void PackageRegistry::wait() {
  impl_.wait();
}

void PackageRegistry::Impl::run() {
  std::vector<std::string> packages;
  
  mutex_.lock();
  packages_.clear();
  mutex_.unlock();
  
  if (ros::package::getAll(packages)) {
    for (size_t i = 0; i < packages.size(); ++i) {
      QString package = QString::fromStdString(packages[i]);
      QDir directory(QString::fromStdString(ros::package::
        getPath(packages[i])));

      if (directory.exists()) {
        mutex_.lock();
        packages_[package] = directory.absolutePath();
        mutex_.unlock();
      }
    }
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PackageRegistry::threadStarted() {
  emit updateStarted();
}
  
void PackageRegistry::threadFinished() {
  emit updateFinished();
}
  
}
