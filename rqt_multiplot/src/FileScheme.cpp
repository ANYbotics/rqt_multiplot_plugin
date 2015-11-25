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

#include "rqt_multiplot/FileScheme.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

FileScheme::FileScheme(QObject* parent, const QString& prefix, const QString&
    rootPath, QDir::Filters filter) :
  UrlScheme(prefix),
  model_(new QFileSystemModel(this)) {
  model_->setRootPath(rootPath);
  model_->setFilter(filter);
  
  connect(model_, SIGNAL(directoryLoaded(const QString&)), this,
    SLOT(modelDirectoryLoaded(const QString&)));
}

FileScheme::~FileScheme() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void FileScheme::setRootPath(const QString& rootPath) {
  model_->setRootPath(rootPath);
}

QString FileScheme::getRootPath() const {
  return model_->rootPath();
}

void FileScheme::setFilter(QDir::Filters filter) {
  model_->setFilter(filter);
}

QDir::Filters FileScheme::getFilter() const {
  return model_->filter();
}

size_t FileScheme::getNumHosts() const {
  return 0;
}

QModelIndex FileScheme::getHostIndex(size_t row) const {
  return QModelIndex();
}

QVariant FileScheme::getHostData(const QModelIndex& index, int role) const {
  return QVariant();
}

size_t FileScheme::getNumPaths(const QModelIndex& hostIndex, const
    QModelIndex& parent) const {
  if (!parent.isValid())
    return 1;
  else if (!parent.parent().isValid())
    return model_->rowCount(model_->index(model_->rootPath()));
  else {
    if (model_->canFetchMore(parent))
      model_->fetchMore(parent);
    
    return model_->rowCount(parent);
  }
}

QModelIndex FileScheme::getPathIndex(const QModelIndex& hostIndex, size_t
    row, const QModelIndex& parent) const {
  if (!parent.isValid())
    return model_->index(model_->rootPath());
  else
    return model_->index(row, 0, parent);
}

QVariant FileScheme::getPathData(const QModelIndex& index, int role) const {
  if (index == model_->index(model_->rootPath())) {
    if ((role == Qt::DisplayRole) || (role == Qt::EditRole))
      return "/";
  }
  else
    return model_->data(index, role);
  
  return QVariant();
}

QString FileScheme::getHost(const QModelIndex& hostIndex) const {
  return QString();
}

QString FileScheme::getPath(const QModelIndex& hostIndex, const QModelIndex&
    pathIndex) const {
  return model_->rootDirectory().relativeFilePath(model_->filePath(
    pathIndex));
}

QString FileScheme::getFilePath(const QModelIndex& hostIndex, const
    QModelIndex& pathIndex) const {
  if (pathIndex.isValid())
    return model_->filePath(pathIndex);
  
  return QString();
}

QString FileScheme::getFilePath(const QString& host, const QString& path)
    const {
  return model_->rootDirectory().absoluteFilePath(path);
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void FileScheme::modelDirectoryLoaded(const QString& path) {
  emit pathLoaded(QString(), model_->rootDirectory().relativeFilePath(path));
}

}
