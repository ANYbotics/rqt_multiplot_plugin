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

#ifndef RQT_MULTIPLOT_URL_SCHEME_H
#define RQT_MULTIPLOT_URL_SCHEME_H

#include <QObject>
#include <QModelIndex>
#include <QString>
#include <QVariant>

namespace rqt_multiplot {
  class UrlScheme :
    public QObject {
  Q_OBJECT
  public:
    UrlScheme(const QString& prefix, QObject* parent = 0);
    virtual ~UrlScheme();

    const QString& getPrefix() const;
    
    virtual size_t getNumHosts() const = 0;
    virtual QModelIndex getHostIndex(size_t row) const = 0;
    virtual QVariant getHostData(const QModelIndex& index, int role)
      const = 0;
      
    virtual size_t getNumPaths(const QModelIndex& hostIndex, const
      QModelIndex& parent = QModelIndex()) const = 0;
    virtual QModelIndex getPathIndex(const QModelIndex& hostIndex,
      size_t row, const QModelIndex& parent = QModelIndex()) const = 0;
    virtual QVariant getPathData(const QModelIndex& index, int role)
      const = 0;
      
    virtual QString getHost(const QModelIndex& hostIndex) const = 0;
    virtual QString getPath(const QModelIndex& hostIndex, const
      QModelIndex& pathIndex) const = 0;
      
    virtual QString getFilePath(const QModelIndex& hostIndex,
      const QModelIndex& pathIndex) const = 0;
    virtual QString getFilePath(const QString& host, const QString&
      path) const = 0;
    
  signals:
    void resetStarted();
    void resetFinished();
    void pathLoaded(const QString& host, const QString& path);

  private:
    QString prefix_;
  };
};

#endif
