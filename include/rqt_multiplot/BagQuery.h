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

#ifndef RQT_MULTIPLOT_BAG_QUERY_H
#define RQT_MULTIPLOT_BAG_QUERY_H

#include <QObject>

#include <variant_topic_tools/MessageDataType.h>
#include <variant_topic_tools/MessageSerializer.h>

#include <rqt_multiplot/Message.h>

namespace rosbag {
  class MessageInstance;
};

namespace rqt_multiplot {
  class BagQuery :
    public QObject {
  Q_OBJECT
  public:
    friend class BagReader;
    
    BagQuery(QObject* parent = 0);
    ~BagQuery();
    
    bool event(QEvent* event);
    
  signals:
    void messageRead(const QString& topic, const Message& message);
    void aboutToBeDestroyed();
  
  private:
    variant_topic_tools::MessageDataType dataType_;
    variant_topic_tools::MessageSerializer serializer_;
    
    void callback(const rosbag::MessageInstance& instance);

#if QT_VERSION >= QT_VERSION_CHECK(5,0,0)
    void disconnectNotify(const QMetaMethod& signal);
#else
    void disconnectNotify(const char* signal);
#endif
  };
};

#endif
