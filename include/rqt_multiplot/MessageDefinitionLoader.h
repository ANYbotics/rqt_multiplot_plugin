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

#ifndef RQT_MULTIPLOT_MESSAGE_DEFINITION_LOADER_H
#define RQT_MULTIPLOT_MESSAGE_DEFINITION_LOADER_H

#include <QMutex>
#include <QObject>
#include <QString>
#include <QThread>

#include <variant_topic_tools/MessageDefinition.h>

namespace rqt_multiplot {
  class MessageDefinitionLoader :
    public QObject {
  Q_OBJECT
  public:
    MessageDefinitionLoader(QObject* parent = 0);
    ~MessageDefinitionLoader();
    
    QString getType() const;
    variant_topic_tools::MessageDefinition getDefinition() const;
    QString getError() const;
    bool isLoading() const;
    
    void load(const QString& type);
    void wait();
    
  signals:
    void loadingStarted();
    void loadingFinished();
    void loadingFailed(const QString& error);
    
  private:
    class Impl :
      public QThread {
    public:
      Impl(QObject* parent = 0);
      virtual ~Impl();
      
      void run();
      
      mutable QMutex mutex_;
      QString type_;
      variant_topic_tools::MessageDefinition definition_;
      QString error_;
    };
    
    Impl impl_;
    
  private slots:
    void threadStarted();
    void threadFinished();
  };
};

#endif
