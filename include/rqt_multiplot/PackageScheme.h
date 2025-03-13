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

#ifndef RQT_MULTIPLOT_PACKAGE_SCHEME_H
#define RQT_MULTIPLOT_PACKAGE_SCHEME_H

#include <QDir>
#include <QFileSystemModel>
#include <QList>
#include <QMap>
#include <QStringListModel>

#include <rqt_multiplot/PackageRegistry.h>
#include <rqt_multiplot/UrlScheme.h>

namespace rqt_multiplot {
class PackageScheme : public UrlScheme {
  Q_OBJECT
 public:
  explicit PackageScheme(QObject* parent = nullptr, const QString& prefix = "package", QDir::Filters filter = QDir::NoFilter);
  ~PackageScheme() override;

  void setFilter(QDir::Filters filter);
  QDir::Filters getFilter() const;

  size_t getNumHosts() const override;
  QModelIndex getHostIndex(size_t row) const override;
  QVariant getHostData(const QModelIndex& index, int role) const override;

  size_t getNumPaths(const QModelIndex& hostIndex, const QModelIndex& parent) const override;
  QModelIndex getPathIndex(const QModelIndex& hostIndex, size_t row, const QModelIndex& parent = QModelIndex()) const override;
  QVariant getPathData(const QModelIndex& index, int role) const override;

  QString getHost(const QModelIndex& hostIndex) const override;
  QString getPath(const QModelIndex& hostIndex, const QModelIndex& pathIndex) const override;

  QString getFilePath(const QModelIndex& hostIndex, const QModelIndex& pathIndex) const override;
  QString getFilePath(const QString& host, const QString& path) const override;

 private:
  PackageRegistry* registry_;

  QFileSystemModel* fileSystemModel_;
  QStringListModel* packageListModel_;

  QList<QString> packages_;
  QMap<QString, QString> packagePaths_;

 private slots:
  void registryUpdateStarted();
  void registryUpdateFinished();

  void modelDirectoryLoaded(const QString& path);
};
}  // namespace rqt_multiplot

#endif
