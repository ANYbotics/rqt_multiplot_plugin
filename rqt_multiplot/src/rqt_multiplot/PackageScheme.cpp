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

#include <ros/package.h>

#include "rqt_multiplot/PackageScheme.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PackageScheme::PackageScheme(QObject* parent, const QString& prefix,
    QDir::Filters filter) :
  UrlScheme(prefix),
  registry_(new PackageRegistry(this)),
  fileSystemModel_(new QFileSystemModel(this)),
  packageListModel_(new QStringListModel(this)) {
  fileSystemModel_->setRootPath("/");
  fileSystemModel_->setFilter(filter);
  
  connect(registry_, SIGNAL(updateStarted()), this,
    SLOT(registryUpdateStarted()));
  connect(registry_, SIGNAL(updateFinished()), this,
    SLOT(registryUpdateFinished()));
  
  connect(fileSystemModel_, SIGNAL(directoryLoaded(const QString&)),
    this, SLOT(modelDirectoryLoaded(const QString&)));
  
  if (registry_->isUpdating())
    registryUpdateStarted();
  else if (registry_->isEmpty())
    registry_->update();
}

PackageScheme::~PackageScheme() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PackageScheme::setFilter(QDir::Filters filter) {
  fileSystemModel_->setFilter(filter);
}

QDir::Filters PackageScheme::getFilter() const {
  return fileSystemModel_->filter();
}

size_t PackageScheme::getNumHosts() const {
  return packages_.count();
}

QModelIndex PackageScheme::getHostIndex(size_t row) const {
  return packageListModel_->index(row);
}

QVariant PackageScheme::getHostData(const QModelIndex& index, int role)
  const {
  return packageListModel_->data(index, role);
}
  
size_t PackageScheme::getNumPaths(const QModelIndex& hostIndex, const
    QModelIndex& parent) const {
  if (!parent.isValid()) {
    if (hostIndex.isValid()) {
      QString packagePath = packagePaths_[packages_[hostIndex.row()]];
      QModelIndex packagePathIndex = fileSystemModel_->index(packagePath);
      
      if (fileSystemModel_->canFetchMore(packagePathIndex))
        fileSystemModel_->fetchMore(packagePathIndex);
      
      return fileSystemModel_->rowCount(packagePathIndex);
    }
  }
  else {
    if (fileSystemModel_->canFetchMore(parent))
      fileSystemModel_->fetchMore(parent);
    
    return fileSystemModel_->rowCount(parent);
  }
  
  return 0;
}

QModelIndex PackageScheme::getPathIndex(const QModelIndex& hostIndex, size_t
    row, const QModelIndex& parent) const {
  if (!parent.isValid()) {
    if (hostIndex.isValid()) {
      QString packagePath = packagePaths_[packages_[hostIndex.row()]];
      QModelIndex packagePathIndex = fileSystemModel_->index(packagePath);
      
      return fileSystemModel_->index(row, 0, packagePathIndex);
    }
  }
  else
    return fileSystemModel_->index(row, 0, parent);
  
  return QModelIndex();
}

QVariant PackageScheme::getPathData(const QModelIndex& index, int role)
    const {
  return fileSystemModel_->data(index, role);
}

QString PackageScheme::getHost(const QModelIndex& hostIndex) const {
  if (hostIndex.isValid())
    return packages_[hostIndex.row()];

  return QString();
}

QString PackageScheme::getPath(const QModelIndex& hostIndex, const
    QModelIndex& pathIndex) const {
  if (hostIndex.isValid()) {
    QString packagePath = packagePaths_[packages_[hostIndex.row()]];
    QString path = fileSystemModel_->filePath(pathIndex);
    
    return QDir(packagePath).relativeFilePath(path);
  }
  
  return QString();
}

QString PackageScheme::getFilePath(const QModelIndex& hostIndex, const
    QModelIndex& pathIndex) const {
  if (hostIndex.isValid()) {
    if (pathIndex.isValid())
      return fileSystemModel_->filePath(pathIndex);
    else
      return packagePaths_[packages_[hostIndex.row()]];
  }
  
  return QString();
}

QString PackageScheme::getFilePath(const QString& host, const QString& path)
    const {
  QString packagePath;
  
  if (!packagePaths_.isEmpty()) {
    QMap<QString, QString>::const_iterator it = packagePaths_.find(host);
    
    if (it != packagePaths_.end())
      packagePath = it.value();
  }
  else
    packagePath = QString::fromStdString(ros::package::getPath(
      host.toStdString()));
    
  if (!packagePath.isEmpty())  {
    QDir packageDir(packagePath);
    
    if (packageDir.exists())
      return packageDir.absoluteFilePath(path);
  }
  
  return QString();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PackageScheme::registryUpdateStarted() {
  emit resetStarted();
}

void PackageScheme::registryUpdateFinished() {
  packagePaths_ = registry_->getPackages();
  packages_ = packagePaths_.keys();
  
  packageListModel_->setStringList(packages_);
  
  emit resetFinished();
}

void PackageScheme::modelDirectoryLoaded(const QString& path) {
  for (QMap<QString, QString>::const_iterator it = packagePaths_.begin();
      it != packagePaths_.end(); ++it) {
    if (path.startsWith(it.value()))
      emit pathLoaded(it.key(), QDir(it.value()).relativeFilePath(path));
  }
}

}
