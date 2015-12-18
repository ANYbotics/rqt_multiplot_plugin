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

#include <QKeyEvent>

#include "rqt_multiplot/CurveListWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveListWidget::CurveListWidget(QWidget* parent) :
  QListWidget(parent) {
}

CurveListWidget::~CurveListWidget() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t CurveListWidget::getNumCurves() const {
  return count();
}

CurveItemWidget* CurveListWidget::getCurveItem(size_t index) const {
  QListWidgetItem* widgetItem = item(index);
  
  if (widgetItem)
    return static_cast<CurveItemWidget*>(itemWidget(widgetItem));
  else
    return 0;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveListWidget::addCurve(CurveConfig* config) {
  CurveItemWidget* itemWidget = new CurveItemWidget(this);
  itemWidget->setConfig(config);
  
  QListWidgetItem* widgetItem = new QListWidgetItem(this);
  widgetItem->setSizeHint(itemWidget->sizeHint());

  addItem(widgetItem);
  setItemWidget(widgetItem, itemWidget);

  emit curveAdded(row(widgetItem));
}

void CurveListWidget::removeCurve(size_t index) {
  QListWidgetItem* widgetItem = item(index);

  if (widgetItem) {
    delete widgetItem;
    
    emit curveRemoved(index);
  }
}

void CurveListWidget::keyPressEvent(QKeyEvent* event) {
  if ((event->modifiers() == Qt::ControlModifier) &&
      (event->key() == Qt::Key_A)) {
    for (size_t index = 0; index < count(); ++index)
      item(index)->setSelected(true);
  }
  
  QListWidget::keyPressEvent(event);
}

}
