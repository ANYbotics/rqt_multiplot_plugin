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

#include <QRegExp>
#include <QStringList>

#include <variant_topic_tools/ArrayDataType.h>
#include <variant_topic_tools/BuiltinDataType.h>
#include <variant_topic_tools/MessageDataType.h>

#include "rqt_multiplot/MessageFieldItem.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldItem::MessageFieldItem(const variant_topic_tools::DataType&
    dataType, MessageFieldItem* parent, const QString& name) :
  parent_(parent),
  name_(name),
  dataType_(dataType) {
  if (dataType_.isMessage()) {
    variant_topic_tools::MessageDataType messageType = dataType_;
    
    for (size_t i = 0; i < messageType.getNumVariableMembers(); ++i)
      appendChild(new MessageFieldItem(messageType.getVariableMember(i).
        getType(), this, QString::fromStdString(messageType.
        getVariableMember(i).getName())));
  }
  else if (dataType_.isArray()) {
    variant_topic_tools::ArrayDataType arrayType = dataType_;
    
    if (!arrayType.isDynamic()) {
      for (size_t i = 0; i < arrayType.getNumMembers(); ++i)
        appendChild(new MessageFieldItem(arrayType.getMemberType(), this,
          QString::number(i)));
    }
    else {
      for (size_t i = 0; i <= 9; ++i)
        appendChild(new MessageFieldItem(arrayType.getMemberType(), this,
          QString::number(i)));
    }
  }
}

MessageFieldItem::~MessageFieldItem() {
  for (QList<MessageFieldItem*>::iterator it = children_.begin();
       it != children_.end(); ++it)
    delete *it;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

MessageFieldItem* MessageFieldItem::getParent() const {
  return parent_;
}

size_t MessageFieldItem::getNumChildren() const {
  return children_.count();
}

MessageFieldItem* MessageFieldItem::getChild(size_t row) const {
  return children_.value(row);
}

MessageFieldItem* MessageFieldItem::getChild(const QString& name) const {
  for (QList<MessageFieldItem*>::const_iterator it = children_.begin();
      it != children_.end(); ++it) {
    if ((*it)->name_ == name)
      return *it;
  }
  
  return 0;
}

MessageFieldItem* MessageFieldItem::getDescendant(const QString& path) const {
  QStringList names = path.split("/");
  
  if (!names.isEmpty()) {
    MessageFieldItem* child = getChild(names.first());
    
    if (child) {
      names.removeFirst();
      return child->getDescendant(names.join("/"));
    }
  }
  
  return 0;
}

int MessageFieldItem::getRow() const {
  if (parent_)
    return parent_->children_.indexOf(const_cast<MessageFieldItem*>(this));

  return -1;
}

size_t MessageFieldItem::getNumColumns() const {
  return 1;
}

const QString& MessageFieldItem::getName() const {
  return name_;
}

const variant_topic_tools::DataType& MessageFieldItem::getDataType() const {
  return dataType_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageFieldItem::appendChild(MessageFieldItem* child) {
  children_.append(child);
}

void MessageFieldItem::update(const QString& path) {
  QStringList names = path.split("/");

  if (dataType_.isArray() && QRegExp("[1-9][0-9]*").exactMatch(
      names.first())) {
    variant_topic_tools::ArrayDataType arrayType = dataType_;
    
    if (arrayType.isDynamic()) {        
      if (children_.count() < 11)
        appendChild(new MessageFieldItem(arrayType.getMemberType(), this));          
      
      children_[0]->name_ = names.first();
      
      for (size_t i = 0; i <= 9; ++i)
        children_[i+1]->name_ = names.first()+QString::number(i);
    }
  }
  
  for (size_t row = 0; row < children_.count(); ++row) {
    MessageFieldItem* child = children_[row];
    
    if (child->dataType_.isArray()) {
      variant_topic_tools::ArrayDataType arrayType = child->dataType_;
      
      if (arrayType.isDynamic()) {
        if (child->children_.count() > 10) {
          for (size_t i = 0; i <= 9; ++i)
            child->children_[i]->name_ = QString::number(i);
          
          delete child->children_.last();        
          child->children_.removeLast();
        }
        
      }
    }
  }
  
  if (!names.isEmpty()) {
    MessageFieldItem* child = getChild(names.first());
    
    if (child) {
      names.removeFirst();
      child->update(names.join("/"));
    }
  }
}

}
