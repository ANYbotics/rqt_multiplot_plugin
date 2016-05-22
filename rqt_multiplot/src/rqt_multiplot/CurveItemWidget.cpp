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

#include <ui_CurveItemWidget.h>

#include "rqt_multiplot/CurveItemWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveItemWidget::CurveItemWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveItemWidget()),
  config_(0) {
  ui_->setupUi(this);

  ui_->frameColor->installEventFilter(this);
}

CurveItemWidget::~CurveItemWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveItemWidget::setConfig(CurveConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(titleChanged(const QString&)), this,
        SLOT(configTitleChanged(const QString&)));
      disconnect(config_->getAxisConfig(CurveConfig::X),
        SIGNAL(changed()), this, SLOT(configXAxisConfigChanged()));
      disconnect(config_->getAxisConfig(CurveConfig::Y),
        SIGNAL(changed()), this, SLOT(configYAxisConfigChanged()));
      disconnect(config_->getColorConfig(), SIGNAL(currentColorChanged(const
        QColor&)), this, SLOT(configColorConfigCurrentColorChanged(const
        QColor&)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(titleChanged(const QString&)), this,
        SLOT(configTitleChanged(const QString&)));
      connect(config->getAxisConfig(CurveConfig::X),
        SIGNAL(changed()), this, SLOT(configXAxisConfigChanged()));
      connect(config->getAxisConfig(CurveConfig::Y),
        SIGNAL(changed()), this, SLOT(configYAxisConfigChanged()));
      connect(config->getColorConfig(), SIGNAL(currentColorChanged(const
        QColor&)), this, SLOT(configColorConfigCurrentColorChanged(const
        QColor&)));
      
      configTitleChanged(config->getTitle());
      configXAxisConfigChanged();
      configYAxisConfigChanged();
      configColorConfigCurrentColorChanged(config->getColorConfig()->
        getCurrentColor());
    }
  }
}

CurveConfig* CurveItemWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool CurveItemWidget::eventFilter(QObject* object, QEvent* event) {
  if (config_) {
    if ((object == ui_->frameColor) && (event->type() == QEvent::Paint)) {
      QPaintEvent* paintEvent = static_cast<QPaintEvent*>(event);
      
      QPainter painter(ui_->frameColor);
      QColor color = config_->getColorConfig()->getCurrentColor();
      
      painter.setBrush(color);
      painter.setPen((color.lightnessF() > 0.5) ? Qt::black : Qt::white);
      
      painter.fillRect(paintEvent->rect(), color);
      painter.drawText(paintEvent->rect(), color.name().toUpper(),
        Qt::AlignHCenter | Qt::AlignVCenter);
    }
  }
  
  return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveItemWidget::configTitleChanged(const QString& title) {
  ui_->labelTitle->setText(config_->getTitle());  
}

void CurveItemWidget::configXAxisConfigChanged() {
  CurveAxisConfig* config = config_->getAxisConfig(CurveConfig::X);
  
  QString text = config->getTopic();
  
  if (config->getFieldType() == CurveAxisConfig::MessageData)
    text += "/"+config->getField();
  else
    text += "/receipt_time";

  ui_->labelXAxis->setText(text);
}

void CurveItemWidget::configYAxisConfigChanged() {
  CurveAxisConfig* config = config_->getAxisConfig(CurveConfig::Y);
  
  QString text = config->getTopic();
  
  if (config->getFieldType() == CurveAxisConfig::MessageData)
    text += "/"+config->getField();
  else
    text += "/receipt_time";

  ui_->labelYAxis->setText(text);
}

void CurveItemWidget::configColorConfigCurrentColorChanged(const QColor&
    color) {
  ui_->frameColor->repaint();
}

}
