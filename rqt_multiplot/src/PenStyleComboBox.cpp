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
#include <QPaintEvent>
#include <QPen>

#include <rqt_multiplot/PenStyleItemDelegate.h>

#include "rqt_multiplot/PenStyleComboBox.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PenStyleComboBox::PenStyleComboBox(QWidget* parent) :
  QComboBox(parent) {
  setItemDelegate(new PenStyleItemDelegate(this));
    
  for (int style = Qt::SolidLine; style < Qt::CustomDashLine; ++style)
    addItem("", style);    
  
  connect(this, SIGNAL(currentIndexChanged(int)), this,
    SLOT(currentIndexChanged(int)));
}

PenStyleComboBox::~PenStyleComboBox() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PenStyleComboBox::setCurrentStyle(Qt::PenStyle style) {
  setCurrentIndex(style-Qt::SolidLine);
}

Qt::PenStyle PenStyleComboBox::getCurrentStyle() const {
  return static_cast<Qt::PenStyle>(Qt::SolidLine+currentIndex());
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PenStyleComboBox::paintEvent(QPaintEvent* event) {
  QComboBox::paintEvent(event);
  
  QVariant data = itemData(currentIndex(), Qt::UserRole);
  
  if (data.isValid()) {
    QPainter painter(this);
    QPen pen;
    
    pen.setColor(palette().color(QPalette::Text));
    pen.setWidth(1);
    pen.setStyle(static_cast<Qt::PenStyle>(data.toInt()));
    
    painter.setPen(pen);
    painter.drawLine(event->rect().left()+5, event->rect().center().y(),
      event->rect().right()-20, event->rect().center().y());
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PenStyleComboBox::currentIndexChanged(int index) {
  if (index >= 0)
    emit currentStyleChanged(static_cast<Qt::PenStyle>(Qt::SolidLine+index));
}

}
