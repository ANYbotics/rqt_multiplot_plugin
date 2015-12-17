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

#include <ui_CurveColorConfigWidget.h>

#include "rqt_multiplot/CurveColorConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveColorConfigWidget::CurveColorConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveColorConfigWidget()),
  config_(0) {
  ui_->setupUi(this);
    
  ui_->labelColor->setAutoFillBackground(true);
  
  connect(ui_->checkBoxAuto, SIGNAL(stateChanged(int)), this,
    SLOT(checkBoxAutoStateChanged(int)));
  
  ui_->labelColor->installEventFilter(this);
}

CurveColorConfigWidget::~CurveColorConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveColorConfigWidget::setConfig(CurveColorConfig* config) {
  if (config_ != config) {
    if (config_) {
      disconnect(config_, SIGNAL(typeChanged(int)), this,
        SLOT(colorTypeChanged(int)));
      disconnect(config_, SIGNAL(currentColorChanged(const QColor&)), this,
        SLOT(colorCurrentColorChanged(const QColor&)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(typeChanged(int)), this,
        SLOT(configTypeChanged(int)));
      connect(config, SIGNAL(currentColorChanged(const QColor&)), this,
        SLOT(configCurrentColorChanged(const QColor&)));
      
      configTypeChanged(config->getType());
      configCurrentColorChanged(config->getCurrentColor());
    }
  }
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool CurveColorConfigWidget::eventFilter(QObject* object, QEvent* event) {
  if ((object == ui_->labelColor) && (ui_->labelColor->isEnabled()) &&
      config_ && (event->type() == QEvent::MouseButtonPress)) {
    QColorDialog dialog(this);
  
    dialog.setCurrentColor(config_->getCustomColor());
  
    if (dialog.exec() == QDialog::Accepted)
      config_->setCustomColor(dialog.currentColor());
  }
  
  return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveColorConfigWidget::configTypeChanged(int type) {
  ui_->checkBoxAuto->setCheckState((type == CurveColorConfig::Auto) ?
    Qt::Checked : Qt::Unchecked);
}

void CurveColorConfigWidget::configCurrentColorChanged(const QColor& color) {
  QPalette palette = ui_->labelColor->palette();
  palette.setColor(QPalette::Window, color);
  palette.setColor(QPalette::WindowText, (color.lightnessF() > 0.5) ?
    Qt::black : Qt::white);
  
  ui_->labelColor->setPalette(palette);
  ui_->labelColor->setText(color.name().toUpper());
}

void CurveColorConfigWidget::checkBoxAutoStateChanged(int state) {
  ui_->labelColor->setEnabled(state != Qt::Checked);
  
  if (config_)
    config_->setType((state == Qt::Checked) ? CurveColorConfig::Auto :
      CurveColorConfig::Custom);
}

}
