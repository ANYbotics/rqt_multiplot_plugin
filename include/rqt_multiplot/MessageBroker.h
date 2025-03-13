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

#ifndef RQT_MULTIPLOT_MESSAGE_BROKER_H
#define RQT_MULTIPLOT_MESSAGE_BROKER_H

#include <QMap>
#include <QObject>
#include <QVariant>

#include <rqt_multiplot/Message.h>

namespace rqt_multiplot {
class MessageBroker : public QObject {
  Q_OBJECT
 public:
  using PropertyMap = QMap<int, QVariant>;

  explicit MessageBroker(QObject* parent = nullptr);
  ~MessageBroker() override;

  virtual bool subscribe(const QString& topic, QObject* receiver, const char* method, const PropertyMap& properties = PropertyMap(),
                         Qt::ConnectionType type = Qt::AutoConnection) = 0;
  virtual bool unsubscribe(const QString& topic, QObject* receiver, const char* method = nullptr) = 0;
};
}  // namespace rqt_multiplot

#endif
