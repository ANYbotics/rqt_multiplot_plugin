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

#ifndef RQT_MULTIPLOT_MESSAGE_SUBSCRIBER_H
#define RQT_MULTIPLOT_MESSAGE_SUBSCRIBER_H

#include <QMap>
#include <QObject>
#include <QString>
#include <QMetaMethod>

#include <ros/node_handle.h>

#include <variant_topic_tools/Subscriber.h>

#include <rqt_multiplot/Message.h>

namespace rqt_multiplot {
  class MessageSubscriber :
    public QObject {
  Q_OBJECT
  public:
    enum Property {
      QueueSize
    };
    
    MessageSubscriber(QObject* parent = 0, const ros::NodeHandle&
      nodeHandle = ros::NodeHandle("~"));
    ~MessageSubscriber();
    
    const ros::NodeHandle& getNodeHandle() const;
    void setTopic(const QString& topic);
    const QString& getTopic() const;
    void setQueueSize(size_t queueSize);
    size_t getQueueSize() const;
    size_t getNumPublishers() const;  
    bool isValid() const;

    bool event(QEvent* event);
    
  signals:
    void subscribed(const QString& topic);
    void messageReceived(const QString& topic, const Message& message);
    void unsubscribed(const QString& topic);
    void aboutToBeDestroyed();
  
  private:
    ros::NodeHandle nodeHandle_;
    
    QString topic_;
    size_t queueSize_;
    
    variant_topic_tools::Subscriber subscriber_;  
      
    void subscribe();
    void unsubscribe();

    void callback(const variant_topic_tools::MessageVariant& variant,
      const ros::Time& receiptTime);

#if QT_VERSION >= QT_VERSION_CHECK(5,0,0)
    void connectNotify(const QMetaMethod& signal);
    void disconnectNotify(const QMetaMethod& signal);
#else
    void connectNotify(const char* signal);
    void disconnectNotify(const char* signal);
#endif
  };
};

#endif
