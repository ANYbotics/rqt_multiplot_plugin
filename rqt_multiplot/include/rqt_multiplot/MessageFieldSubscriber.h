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

#ifndef RQT_MULTIPLOT_MESSAGE_FIELD_SUBSCRIBER_H
#define RQT_MULTIPLOT_MESSAGE_FIELD_SUBSCRIBER_H

#include <QObject>
#include <QString>

#include <rqt_multiplot/Message.h>
#include <rqt_multiplot/MessageSubscriber.h>
#include <rqt_multiplot/MessageSubscriberRegistry.h>

namespace rqt_multiplot {
  class MessageFieldSubscriber :
    public QObject {
  Q_OBJECT
  public:
    MessageFieldSubscriber(QObject* parent = 0);
    ~MessageFieldSubscriber();

    void setTopic(const QString& topic);
    const QString& getTopic() const;
    void setField(const QString& field);
    const QString& getField() const;
    void setQueueSize(size_t queueSize);
    size_t getQueueSize() const;
    size_t getNumPublishers() const;  
    bool isValid() const;
    
  signals:
    void subscribed(const QString& topic, const QString& field);
    void valueReceived(const QString& topic, const QString& field,
      double value);
    void unsubscribed(const QString& topic, const QString& field);
    void aboutToBeDestroyed();
    
  private:
    QString topic_;
    QString field_;
    size_t queueSize_;
    
    MessageSubscriberRegistry* subscriberRegistry_;
    MessageSubscriber* subscriber_;
    
    void subscribe();
    void unsubscribe();
    
    void connectNotify(const char* signal);
    void disconnectNotify(const char* signal);
    
  private slots:
    void messageReceived(const QString& topic, const Message& message);
  };
};

#endif
