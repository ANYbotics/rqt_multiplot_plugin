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

#include <QPainter>

#include "rqt_multiplot/PenStyleItemDelegate.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PenStyleItemDelegate::PenStyleItemDelegate(QWidget* parent) :
  QItemDelegate(parent) {
}

PenStyleItemDelegate::~PenStyleItemDelegate() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PenStyleItemDelegate::paint(QPainter* painter, const
    QStyleOptionViewItem& option, const QModelIndex& index) const {
  QVariant data = index.model()->data(index, Qt::UserRole);

  if (option.state & QStyle::State_Selected)
    painter->fillRect(option.rect, option.palette.highlight());
  
  if (data.isValid()) {
    painter->save();

    QPen pen = painter->pen();

    if (option.state & QStyle::State_Selected)
      pen.setColor(option.palette.color(QPalette::HighlightedText));
    else
      pen.setColor(option.palette.color(QPalette::Text));
    
    pen.setWidth(1);
    pen.setStyle(static_cast<Qt::PenStyle>(data.toInt()));
    
    painter->setPen(pen);
    painter->drawLine(option.rect.left(), option.rect.center().y(),
      option.rect.right(), option.rect.center().y());

    painter->restore();
  }
  else
    QItemDelegate::paint(painter, option, index);
}

}
