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

#include <QMutexLocker>

#include <rqt_multiplot/DataTypeRegistry.h>

#include "rqt_multiplot/MessageDefinitionLoader.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageDefinitionLoader::MessageDefinitionLoader(QObject* parent) :
  QObject(parent),
  impl_(this) {
  connect(&impl_, SIGNAL(started()), this, SLOT(threadStarted()));
  connect(&impl_, SIGNAL(finished()), this, SLOT(threadFinished()));
}

MessageDefinitionLoader::~MessageDefinitionLoader() {
  impl_.quit();
  impl_.wait();
}

MessageDefinitionLoader::Impl::Impl(QObject* parent) :
  QThread(parent) {
}

MessageDefinitionLoader::Impl::~Impl() {
  terminate();
  wait();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

QString MessageDefinitionLoader::getType() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.type_;
}

variant_topic_tools::MessageDefinition MessageDefinitionLoader::
    getDefinition() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.definition_;
}

QString MessageDefinitionLoader::getError() const {
  QMutexLocker lock(&impl_.mutex_);
  
  return impl_.error_;
}

bool MessageDefinitionLoader::isLoading() const {
  return impl_.isRunning();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageDefinitionLoader::load(const QString& type) {
  impl_.wait();
  
  impl_.type_ = type;
  impl_.start();
}

void MessageDefinitionLoader::wait() {
  impl_.wait();
}

void MessageDefinitionLoader::Impl::run() {
  QMutexLocker lock(&mutex_);
  
  error_.clear();
  
  try {
    QMutexLocker lock(&DataTypeRegistry::mutex_);
    
    definition_.load(type_.toStdString());
  }
  catch (const ros::Exception& exception) {
    definition_.clear();
    error_ = QString::fromStdString(exception.what());
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageDefinitionLoader::threadStarted() {
  emit loadingStarted();
}
  
void MessageDefinitionLoader::threadFinished() {
  if (impl_.error_.isEmpty())
    emit loadingFinished();
  else
    emit loadingFailed(impl_.error_);
}
  
}
