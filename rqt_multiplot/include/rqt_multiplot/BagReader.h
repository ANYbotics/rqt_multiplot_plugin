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

#ifndef RQT_MULTIPLOT_BAG_READER_H
#define RQT_MULTIPLOT_BAG_READER_H

#include <QMap>
#include <QMutex>
#include <QString>
#include <QStringList>
#include <QThread>

#include <rqt_multiplot/BagQuery.h>
#include <rqt_multiplot/MessageBroker.h>

namespace rqt_multiplot {
  class BagReader :
    public MessageBroker {
  Q_OBJECT
  public:
    BagReader(QObject* parent = 0);
    virtual ~BagReader();
    
    QString getFileName() const;
    QString getError() const;
    bool isReading() const;
    
    void read(const QString& fileName);
    void wait();
    
    bool subscribe(const QString& topic, QObject* receiver,
      const char* method, const PropertyMap& properties = PropertyMap(),
      Qt::ConnectionType type = Qt::AutoConnection);
    bool unsubscribe(const QString& topic, QObject* receiver,
      const char* method = 0);
    
    bool event(QEvent* event);
    
  signals:
    void readingStarted();
    void messageRead(const QString& topic, const Message& message);
    void readingProgressChanged(double progress);
    void readingFinished();
    void readingFailed(const QString& error);
    
  private:
    class Impl :
      public QThread {
    public:
      Impl(QObject* parent = 0);
      virtual ~Impl();
      
      void run();
      
      QMutex mutex_;
      QString fileName_;
      QString error_;
      
      QMap<QString, BagQuery*> queries_;
    };
    
    Impl impl_;
    
  private slots:
    void threadStarted();
    void threadFinished();

    void queryAboutToBeDestroyed();
  };
};

#endif
