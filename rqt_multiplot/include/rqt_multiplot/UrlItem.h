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

#ifndef RQT_MULTIPLOT_URL_ITEM_H
#define RQT_MULTIPLOT_URL_ITEM_H

#include <QMap>
#include <QModelIndex>

#include <rqt_multiplot/UrlScheme.h>

namespace rqt_multiplot {
  class UrlItem {
  public:
    enum Type {
      Scheme,
      Host,
      Path
    };
    
    UrlItem(UrlScheme* scheme = 0, Type type = Scheme, const QModelIndex&
      index = QModelIndex(), UrlItem* parent = 0);
    ~UrlItem();
  
    UrlItem* getParent() const;
    size_t getNumChildren() const;
    UrlItem* getChild(size_t row) const;
    int getRow() const;
    
    void setScheme(UrlScheme* scheme);
    UrlScheme* getScheme() const;
    void setType(Type type);
    Type getType() const;
    void setIndex(const QModelIndex& index);
    const QModelIndex& getIndex() const;
    QModelIndex getIndex(Type type) const;

    UrlItem* addChild(size_t row, Type type, const QModelIndex& index);
    
  private:
    UrlItem* parent_;
    QMap<size_t, UrlItem*> children_;
    
    UrlScheme* scheme_;
    Type type_;
    QModelIndex index_;
  };
};

#endif
