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

#include <QColorDialog>

#include <ui_CurveColorWidget.h>

#include "rqt_multiplot/CurveColorWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveColorWidget::CurveColorWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveColorWidget()),
  color_(0) {
  ui_->setupUi(this);
    
  ui_->labelColor->setAutoFillBackground(true);
  
  connect(ui_->checkBoxAuto, SIGNAL(stateChanged(int)), this,
    SLOT(checkBoxAutoStateChanged(int)));
  
  ui_->labelColor->installEventFilter(this);
}

CurveColorWidget::~CurveColorWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveColorWidget::setColor(CurveColor* color) {
  if (color_ != color) {
    if (color_) {
      disconnect(color_, SIGNAL(typeChanged(int)), this,
        SLOT(colorTypeChanged(int)));
      disconnect(color_, SIGNAL(currentColorChanged(const QColor&)), this,
        SLOT(colorCurrentColorChanged(const QColor&)));
    }
    
    color_ = color;
    
    if (color) {
      connect(color, SIGNAL(typeChanged(int)), this,
        SLOT(colorTypeChanged(int)));
      connect(color, SIGNAL(currentColorChanged(const QColor&)), this,
        SLOT(colorCurrentColorChanged(const QColor&)));
      
      colorTypeChanged(color->getType());
      colorCurrentColorChanged(color->getCurrentColor());
    }
  }
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool CurveColorWidget::eventFilter(QObject* object, QEvent* event) {
  if ((object == ui_->labelColor) && (ui_->labelColor->isEnabled()) &&
      color_ && (event->type() == QEvent::MouseButtonPress)) {
    QColorDialog dialog(this);
  
    dialog.setCurrentColor(color_->getCustomColor());
  
    if (dialog.exec() == QDialog::Accepted)
      color_->setCustomColor(dialog.currentColor());
  }
  
  return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveColorWidget::colorTypeChanged(int type) {
  ui_->checkBoxAuto->setCheckState((type == CurveColor::Auto) ?
    Qt::Checked : Qt::Unchecked);
}

void CurveColorWidget::colorCurrentColorChanged(const QColor& color) {
  QPalette palette = ui_->labelColor->palette();
  palette.setColor(QPalette::Window, color);
  palette.setColor(QPalette::WindowText, (color.lightnessF() > 0.5) ?
    Qt::black : Qt::white);
  
  ui_->labelColor->setPalette(palette);
  ui_->labelColor->setText(color.name().toUpper());
}

void CurveColorWidget::checkBoxAutoStateChanged(int state) {
  ui_->labelColor->setEnabled(state != Qt::Checked);
  
  if (color_)
    color_->setType((state == Qt::Checked) ? CurveColor::Auto :
      CurveColor::Custom);
}

}
