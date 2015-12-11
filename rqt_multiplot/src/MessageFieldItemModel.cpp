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

#include <variant_topic_tools/MessageVariable.h>

#include "rqt_multiplot/MessageFieldItemModel.h"

Q_DECLARE_METATYPE(variant_topic_tools::DataType)

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldItemModel::MessageFieldItemModel(QObject* parent) :
  QAbstractItemModel(parent),
  rootItem_(0) {
}

MessageFieldItemModel::~MessageFieldItemModel() {
  if (rootItem_)
    delete rootItem_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageFieldItemModel::setMessageDataType(const variant_topic_tools::
    MessageDataType& dataType) {
  if (rootItem_) {
    delete rootItem_;
    rootItem_ = 0;
  }
  
  if (dataType.isValid())
    rootItem_ = new MessageFieldItem(dataType);
}

variant_topic_tools::MessageDataType MessageFieldItemModel::
    getMessageDataType() const {
  if (rootItem_)
    return rootItem_->getDataType();
  else
    return variant_topic_tools::MessageDataType();
}

variant_topic_tools::DataType MessageFieldItemModel::getFieldDataType(const
    QString& field) const {
  if (rootItem_) {
    MessageFieldItem* descendant = rootItem_->getDescendant(field);
    
    if (descendant)
      return descendant->getDataType();
  }
  
  return variant_topic_tools::DataType();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

int MessageFieldItemModel::rowCount(const QModelIndex& parent) const {
  if (parent.column() <= 0) {
    MessageFieldItem* parentItem = 0;
  
    if (!parent.isValid())
      parentItem = rootItem_;
    else
      parentItem = static_cast<MessageFieldItem*>(parent.internalPointer());

    if (parentItem)
      return parentItem->getNumChildren();
  }

  return 0;
}

int MessageFieldItemModel::columnCount(const QModelIndex& parent) const {
  if (parent.isValid()) {
    MessageFieldItem* parentItem = static_cast<MessageFieldItem*>(
      parent.internalPointer());
    
    if (parentItem)
      return parentItem->getNumColumns();
  }
  else if (rootItem_)
    return rootItem_->getNumColumns();
  

  return 0;
}

QVariant MessageFieldItemModel::data(const QModelIndex& index, int role)
    const {
  if (index.isValid()) {
    if ((role == Qt::DisplayRole) || (role == Qt::EditRole)) {
      MessageFieldItem* item = static_cast<MessageFieldItem*>(
        index.internalPointer());

      if (item)
        return item->getName();
    }
  }

  return QVariant();
}

QModelIndex MessageFieldItemModel::index(int row, int column, const
    QModelIndex& parent) const {
  if (hasIndex(row, column, parent)) {
    MessageFieldItem* parentItem = 0;

    if (!parent.isValid())
      parentItem = rootItem_;
    else
      parentItem = static_cast<MessageFieldItem*>(parent.internalPointer());

    if (parentItem) {
      MessageFieldItem* childItem = parentItem->getChild(row);
      
      if (childItem)
        return createIndex(row, column, childItem);
    }
  }

  return QModelIndex();
}

QModelIndex MessageFieldItemModel::parent(const QModelIndex& index) const {
  if (index.isValid()) {
    MessageFieldItem* childItem = static_cast<MessageFieldItem*>(
      index.internalPointer());

    if (childItem) {
      MessageFieldItem* parentItem = childItem->getParent();

      if (parentItem != rootItem_)
        return createIndex(parentItem->getRow(), 0, parentItem);
    }
  }
  
  return QModelIndex();
}

void MessageFieldItemModel::update(const QString& path) {
  if (rootItem_)
    rootItem_->update(path);    
}

}
