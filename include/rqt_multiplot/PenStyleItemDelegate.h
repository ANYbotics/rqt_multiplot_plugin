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

#ifndef RQT_MULTIPLOT_PEN_STYLE_ITEM_DELEGATE_H
#define RQT_MULTIPLOT_PEN_STYLE_ITEM_DELEGATE_H

#include <QItemDelegate>

namespace rqt_multiplot {
  class PenStyleItemDelegate :
    public QItemDelegate {
  Q_OBJECT
  public:
    PenStyleItemDelegate(QWidget* parent = 0);
    virtual ~PenStyleItemDelegate();
    
    void paint(QPainter* painter, const QStyleOptionViewItem& option,
      const QModelIndex& index) const;
  };
};

#endif
