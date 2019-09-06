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

#ifndef RQT_MULTIPLOT_PACKAGE_REGISTRY_H
#define RQT_MULTIPLOT_PACKAGE_REGISTRY_H

#include <QMap>
#include <QMutex>
#include <QObject>
#include <QString>
#include <QThread>

namespace rqt_multiplot {
  class PackageRegistry :
    public QObject {
  Q_OBJECT
  public:
    PackageRegistry(QObject* parent = 0);
    ~PackageRegistry();
    
    QMap<QString, QString> getPackages() const;
    bool isUpdating() const;
    bool isEmpty() const;
    
    void update();
    void wait();
    
  signals:
    void updateStarted();
    void updateFinished();
    
  private:
    class Impl :
      public QThread {
    public:
      Impl(QObject* parent = 0);
      virtual ~Impl();
      
      void run();
      
      mutable QMutex mutex_;
      QMap<QString, QString> packages_;
    };
    
    static Impl impl_;
    
  private slots:
    void threadStarted();
    void threadFinished();
  };
};

#endif
