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

#include "rqt_multiplot/UrlItem.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

UrlItem::UrlItem(UrlScheme* scheme, Type type, const QModelIndex& index,
    UrlItem* parent) :
  parent_(parent),
  scheme_(scheme),
  type_(type),
  index_(index) {
}

UrlItem::~UrlItem() {
  for (QMap<size_t, UrlItem*>::iterator it = children_.begin();
      it != children_.end(); ++it)
    delete it.value();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

UrlItem* UrlItem::getParent() const {
  return parent_;
}

size_t UrlItem::getNumChildren() const {
  return children_.count();
}

UrlItem* UrlItem::getChild(size_t row) const {
  QMap<size_t, UrlItem*>::const_iterator it = children_.find(row);
  
  if (it != children_.end())
    return it.value();
  else
    return 0;
}

int UrlItem::getRow() const {
  if (parent_) {
    for (QMap<size_t, UrlItem*>::const_iterator it = parent_->children_.
        begin(); it != parent_->children_.end(); ++it) {
      if (it.value() == this)
        return it.key();
    }
  }

  return -1;
}

void UrlItem::setScheme(UrlScheme* scheme) {
  scheme_ = scheme;
}

UrlScheme* UrlItem::getScheme() const {
  return scheme_;
}

void UrlItem::setType(Type type) {
  type_ = type;
}

UrlItem::Type UrlItem::getType() const {
  return type_;
}

void UrlItem::setIndex(const QModelIndex& index) {
  index_ = index;
}

const QModelIndex& UrlItem::getIndex() const {
  return index_;
}

QModelIndex UrlItem::getIndex(Type type) const {
  const UrlItem* item = this;
  
  while (item) {
    if (item->type_ != type)
      item = item->parent_;
    else
      return item->index_;
  }
  
  return QModelIndex();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

UrlItem* UrlItem::addChild(size_t row, Type type, const QModelIndex& index) {
  QMap<size_t, UrlItem*>::iterator it = children_.find(row);
  
  if (it != children_.end()) {
    it.value()->type_ = type;
    it.value()->index_ = index;

    return it.value();
  }
  else {
    UrlItem* item = new UrlItem(scheme_, type, index, this);
    children_.insert(row, item);
    
    return item;
  }
}

}
