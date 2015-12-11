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

#ifndef RQT_MULTIPLOT_MESSAGE_FIELD_ITEM_MODEL_H
#define RQT_MULTIPLOT_MESSAGE_FIELD_ITEM_MODEL_H

#include <QAbstractItemModel>

#include <rqt_multiplot/MessageFieldItem.h>

#include <variant_topic_tools/MessageDataType.h>

namespace rqt_multiplot {
  class MessageFieldItemModel :
    public QAbstractItemModel {
  Q_OBJECT
  public:
    MessageFieldItemModel(QObject* parent = 0);
    virtual ~MessageFieldItemModel();
  
    void setMessageDataType(const variant_topic_tools::MessageDataType&
      dataType);
    variant_topic_tools::MessageDataType getMessageDataType() const;
    variant_topic_tools::DataType getFieldDataType(const QString& field)
      const;
    
    int rowCount(const QModelIndex& parent) const;
    int columnCount(const QModelIndex& parent) const;
    QVariant data(const QModelIndex& index, int role) const;
    QModelIndex index(int row, int column, const QModelIndex& parent) const;
    QModelIndex parent(const QModelIndex& index) const;
    
    void update(const QString& path);
    
  private:
    MessageFieldItem* rootItem_;
  };
};

#endif
