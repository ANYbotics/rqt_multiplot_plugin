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

#include <ui_CurveConfigWidget.h>

#include "rqt_multiplot/CurveConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveConfigWidget::CurveConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveConfigWidget()) {
  ui_->setupUi(this);
  
  messageTopicRegistry.update();
  messageTypeRegistry.update();
}

CurveConfigWidget::~CurveConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveConfigWidget::setTitle(const QString& title) {
  ui_->lineEditTitle->setText(title);
}

QString CurveConfigWidget::getTitle() const {
  return ui_->lineEditTitle->text();
}

void CurveConfigWidget::setColor(const QColor& color) {
  QPalette palette;
  palette.setColor(QPalette::Window, color);
  
  ui_->frameColor->setPalette(palette);
}

QColor CurveConfigWidget::getColor() const {
  return ui_->frameColor->palette().color(QPalette::Window);
}

}
