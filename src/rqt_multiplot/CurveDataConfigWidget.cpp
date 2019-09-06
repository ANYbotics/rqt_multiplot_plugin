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

#include <ui_CurveDataConfigWidget.h>

#include "rqt_multiplot/CurveDataConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveDataConfigWidget::CurveDataConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveDataConfigWidget()),
  config_(0) {
  ui_->setupUi(this);
  
  ui_->spinBoxCircularBufferCapacity->setEnabled(false);
  ui_->doubleSpinBoxTimeFrameLength->setEnabled(false);
  
  connect(ui_->radioButtonVector, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonVectorToggled(bool)));
  connect(ui_->radioButtonList, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonListToggled(bool)));
  connect(ui_->radioButtonCircularBuffer, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonCircularBufferToggled(bool)));
  connect(ui_->radioButtonTimeFrame, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonTimeFrameToggled(bool)));
  
  connect(ui_->spinBoxCircularBufferCapacity, SIGNAL(valueChanged(int)),
    this, SLOT(spinBoxCircularBufferCapacityValueChanged(int)));
  connect(ui_->doubleSpinBoxTimeFrameLength, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxTimeFrameLengthValueChanged(double)));
}

CurveDataConfigWidget::~CurveDataConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveDataConfigWidget::setConfig(CurveDataConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(typeChanged(int)), this,
        SLOT(configTypeChanged(int)));
      disconnect(config_, SIGNAL(circularBufferCapacityChanged(size_t)),
        this, SLOT(configCircularBufferCapacityChanged(size_t)));
      disconnect(config_, SIGNAL(timeFrameLengthChanged(double)),
        this, SLOT(configTimeFrameLengthChanged(double)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(typeChanged(int)), this,
        SLOT(configTypeChanged(int)));
      connect(config, SIGNAL(circularBufferCapacityChanged(size_t)),
        this, SLOT(configCircularBufferCapacityChanged(size_t)));
      connect(config, SIGNAL(timeFrameLengthChanged(double)),
        this, SLOT(configTimeFrameLengthChanged(double)));
      
      configTypeChanged(config->getType());
      configCircularBufferCapacityChanged(config->
        getCircularBufferCapacity());
      configTimeFrameLengthChanged(config->getTimeFrameLength());
    }
  }
}

CurveDataConfig* CurveDataConfigWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveDataConfigWidget::configTypeChanged(int type) {
  if (type == CurveDataConfig::List)
    ui_->radioButtonList->setChecked(true);
  else if (type == CurveDataConfig::CircularBuffer)
    ui_->radioButtonCircularBuffer->setChecked(true);
  else if (type == CurveDataConfig::TimeFrame)
    ui_->radioButtonTimeFrame->setChecked(true);
  else
    ui_->radioButtonVector->setChecked(true);
}

void CurveDataConfigWidget::configCircularBufferCapacityChanged(size_t
    capacity) {
  ui_->spinBoxCircularBufferCapacity->setValue(capacity);
}

void CurveDataConfigWidget::configTimeFrameLengthChanged(double length) {
  ui_->doubleSpinBoxTimeFrameLength->setValue(length);
}

void CurveDataConfigWidget::radioButtonVectorToggled(bool checked) {
  if (config_ && checked)
    config_->setType(CurveDataConfig::Vector);
}

void CurveDataConfigWidget::radioButtonListToggled(bool checked) {
  if (config_ && checked)
    config_->setType(CurveDataConfig::List);
}

void CurveDataConfigWidget::radioButtonCircularBufferToggled(bool checked) {
  ui_->spinBoxCircularBufferCapacity->setEnabled(checked);
  
  if (config_ && checked)
    config_->setType(CurveDataConfig::CircularBuffer);
}

void CurveDataConfigWidget::radioButtonTimeFrameToggled(bool checked) {
  ui_->doubleSpinBoxTimeFrameLength->setEnabled(checked);

  if (config_ && checked)
    config_->setType(CurveDataConfig::TimeFrame);
}

void CurveDataConfigWidget::spinBoxCircularBufferCapacityValueChanged(
    int value) {
  if (config_)
    config_->setCircularBufferCapacity(value);
}

void CurveDataConfigWidget::doubleSpinBoxTimeFrameLengthValueChanged(
    double value) {
  if (config_) {
    config_->setTimeFrameLength(value);
  }
}

}
