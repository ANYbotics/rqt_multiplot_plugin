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

#ifndef RQT_MULTIPLOT_FILE_SCHEME_H
#define RQT_MULTIPLOT_FILE_SCHEME_H

#include <QDir>
#include <QFileSystemModel>

#include <rqt_multiplot/UrlScheme.h>

namespace rqt_multiplot {
  class FileScheme :
    public UrlScheme {
  Q_OBJECT
  public:
    FileScheme(QObject* parent = 0, const QString& prefix = "file", const
      QString& rootPath = "/", QDir::Filters filter = QDir::NoFilter);
    virtual ~FileScheme();

    void setRootPath(const QString& rootPath);
    QString getRootPath() const;
    void setFilter(QDir::Filters filter);
    QDir::Filters getFilter() const;

    size_t getNumHosts() const;
    QModelIndex getHostIndex(size_t row) const;
    QVariant getHostData(const QModelIndex& index, int role) const;
      
    size_t getNumPaths(const QModelIndex& hostIndex, const QModelIndex&
      parent) const;
    QModelIndex getPathIndex(const QModelIndex& hostIndex, size_t row,
      const QModelIndex& parent = QModelIndex()) const;
    QVariant getPathData(const QModelIndex& index, int role) const;
      
    QString getHost(const QModelIndex& hostIndex) const;
    QString getPath(const QModelIndex& hostIndex, const QModelIndex&
      pathIndex) const;
    
    QString getFilePath(const QModelIndex& hostIndex, const QModelIndex&
      pathIndex) const;
    QString getFilePath(const QString& host, const QString& path) const;
      
  private:
    QFileSystemModel* model_;
    
  private slots:
    void modelDirectoryLoaded(const QString& path);
  };
};

#endif
