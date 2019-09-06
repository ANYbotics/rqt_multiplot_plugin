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

#ifndef RQT_MULTIPLOT_CURVE_DATA_SEQUENCER_H
#define RQT_MULTIPLOT_CURVE_DATA_SEQUENCER_H

#include <QLinkedList>
#include <QMap>
#include <QObject>
#include <QPointF>
#include <QVector>

#include <ros/time.h>

#include <rqt_multiplot/CurveConfig.h>
#include <rqt_multiplot/MessageBroker.h>

namespace rqt_multiplot {
  class CurveDataSequencer :
    public QObject {
  Q_OBJECT
  public:
    CurveDataSequencer(QObject* parent = 0);
    virtual ~CurveDataSequencer();
    
    void setConfig(CurveConfig* config);
    CurveConfig* getConfig() const;
    void setBroker(MessageBroker* broker);
    MessageBroker* getBroker() const;
    bool isSubscribed() const;
    
    void subscribe();
    void unsubscribe();
    
  signals:
    void subscribed();
    void pointReceived(const QPointF& point);
    void unsubscribed();
    
  private:
    class TimeValue {
    public:
      inline TimeValue(const ros::Time& time = ros::Time(),
          double value = 0.0) :
        time_(time),
        value_(value) {
      };

      inline TimeValue(const TimeValue& src) :
        time_(src.time_),
        value_(src.value_) {
      };
      
      inline bool operator==(const TimeValue& timeValue) const {
        return (time_ == timeValue.time_);
      };
      
      inline bool operator>(const TimeValue& timeValue) const {
        return (time_ > timeValue.time_);
      };
      
      inline bool operator<(const TimeValue& timeValue) const {
        return (time_ < timeValue.time_);
      };
      
      ros::Time time_;
      double value_;
    };
    
    typedef QLinkedList<TimeValue> TimeValueList;
    
    CurveConfig* config_;
    
    MessageBroker* broker_;

    QMap<CurveConfig::Axis, QString> subscribedTopics_;    
    QMap<CurveConfig::Axis, QString> timeFields_;
    QMap<CurveConfig::Axis, TimeValueList> timeValues_;
    
    void processMessage(const Message& message);
    void processMessage(CurveConfig::Axis axis, const Message& message);
    void interpolate();
    
  private slots:
    void configAxisConfigChanged();
    void configSubscriberQueueSizeChanged(size_t queueSize);
    
    void subscriberMessageReceived(const QString& topic, const
      Message& message);
    void subscriberXAxisMessageReceived(const QString& topic, const
      Message& message);
    void subscriberYAxisMessageReceived(const QString& topic, const
      Message& message);
  };
};

#endif
