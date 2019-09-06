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

#include <QDoubleValidator>

#include <ui_CurveAxisScaleConfigWidget.h>

#include "rqt_multiplot/CurveAxisScaleConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveAxisScaleConfigWidget::CurveAxisScaleConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveAxisScaleConfigWidget()),
  config_(0) {
  ui_->setupUi(this);
  
  ui_->lineEditAbsoluteMinimum->setEnabled(false);
  ui_->lineEditAbsoluteMaximum->setEnabled(false);
  ui_->lineEditRelativeMinimum->setEnabled(false);
  ui_->lineEditRelativeMaximum->setEnabled(false);
  
  ui_->lineEditAbsoluteMinimum->setValidator(
    new QDoubleValidator(ui_->lineEditAbsoluteMinimum));
  ui_->lineEditAbsoluteMaximum->setValidator(
    new QDoubleValidator(ui_->lineEditAbsoluteMaximum));
  ui_->lineEditRelativeMinimum->setValidator(
    new QDoubleValidator(ui_->lineEditRelativeMinimum));
  ui_->lineEditRelativeMaximum->setValidator(
    new QDoubleValidator(ui_->lineEditRelativeMaximum));
  
  connect(ui_->radioButtonAbsolute, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonAbsoluteToggled(bool)));
  connect(ui_->radioButtonRelative, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonRelativeToggled(bool)));
  connect(ui_->radioButtonAuto, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonAutoToggled(bool)));
  
  connect(ui_->lineEditAbsoluteMinimum, SIGNAL(editingFinished()),
    this, SLOT(lineEditAbsoluteMinimumEditingFinished()));
  connect(ui_->lineEditAbsoluteMaximum, SIGNAL(editingFinished()),
    this, SLOT(lineEditAbsoluteMaximumEditingFinished()));
  connect(ui_->lineEditRelativeMinimum, SIGNAL(editingFinished()),
    this, SLOT(lineEditRelativeMinimumEditingFinished()));
  connect(ui_->lineEditRelativeMaximum, SIGNAL(editingFinished()),
    this, SLOT(lineEditRelativeMaximumEditingFinished()));
}

CurveAxisScaleConfigWidget::~CurveAxisScaleConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveAxisScaleConfigWidget::setConfig(CurveAxisScaleConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(typeChanged(int)), this,
        SLOT(configTypeChanged(int)));
      disconnect(config_, SIGNAL(absoluteMinimumChanged(double)), this,
        SLOT(configAbsoluteMinimumChanged(double)));
      disconnect(config_, SIGNAL(absoluteMaximumChanged(double)), this,
        SLOT(configAbsoluteMaximumChanged(double)));
      disconnect(config_, SIGNAL(relativeMinimumChanged(double)), this,
        SLOT(configRelativeMinimumChanged(double)));
      disconnect(config_, SIGNAL(relativeMaximumChanged(double)), this,
        SLOT(configRelativeMaximumChanged(double)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(typeChanged(int)), this,
        SLOT(configTypeChanged(int)));
      connect(config, SIGNAL(absoluteMinimumChanged(double)), this,
        SLOT(configAbsoluteMinimumChanged(double)));
      connect(config, SIGNAL(absoluteMaximumChanged(double)), this,
        SLOT(configAbsoluteMaximumChanged(double)));
      connect(config, SIGNAL(relativeMinimumChanged(double)), this,
        SLOT(configRelativeMinimumChanged(double)));
      connect(config, SIGNAL(relativeMaximumChanged(double)), this,
        SLOT(configRelativeMaximumChanged(double)));
      
      configTypeChanged(config->getType());
      configAbsoluteMinimumChanged(config->getAbsoluteMinimum());
      configAbsoluteMaximumChanged(config->getAbsoluteMaximum());
      configRelativeMinimumChanged(config->getRelativeMinimum());
      configRelativeMaximumChanged(config->getRelativeMaximum());
    }
  }
}

CurveAxisScaleConfig* CurveAxisScaleConfigWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveAxisScaleConfigWidget::configTypeChanged(int type) {
  if (type == CurveAxisScaleConfig::Absolute)
    ui_->radioButtonAbsolute->setChecked(true);
  else if (type == CurveAxisScaleConfig::Relative)
    ui_->radioButtonRelative->setChecked(true);
  else
    ui_->radioButtonAuto->setChecked(true);
}

void CurveAxisScaleConfigWidget::configAbsoluteMinimumChanged(double minimum) {
  ui_->lineEditAbsoluteMinimum->setText(QString::number(minimum));
}

void CurveAxisScaleConfigWidget::configAbsoluteMaximumChanged(double maximum) {
  ui_->lineEditAbsoluteMaximum->setText(QString::number(maximum));
}

void CurveAxisScaleConfigWidget::configRelativeMinimumChanged(double minimum) {
  ui_->lineEditRelativeMinimum->setText(QString::number(minimum));
}

void CurveAxisScaleConfigWidget::configRelativeMaximumChanged(double maximum) {
  ui_->lineEditRelativeMaximum->setText(QString::number(maximum));
}

void CurveAxisScaleConfigWidget::radioButtonAbsoluteToggled(bool checked) {
  ui_->lineEditAbsoluteMinimum->setEnabled(checked);
  ui_->lineEditAbsoluteMaximum->setEnabled(checked);
  
  if (config_ && checked)
    config_->setType(CurveAxisScaleConfig::Absolute);
}

void CurveAxisScaleConfigWidget::radioButtonRelativeToggled(bool checked) {
  ui_->lineEditRelativeMinimum->setEnabled(checked);
  ui_->lineEditRelativeMaximum->setEnabled(checked);
  
  if (config_ && checked)
    config_->setType(CurveAxisScaleConfig::Relative);
}

void CurveAxisScaleConfigWidget::radioButtonAutoToggled(bool checked) {
  if (config_ && checked)
    config_->setType(CurveAxisScaleConfig::Auto);
}

void CurveAxisScaleConfigWidget::lineEditAbsoluteMinimumEditingFinished() {
  if (config_)
    config_->setAbsoluteMinimum(ui_->lineEditAbsoluteMinimum->text().
      toDouble());
}

void CurveAxisScaleConfigWidget::lineEditAbsoluteMaximumEditingFinished() {
  if (config_)
    config_->setAbsoluteMaximum(ui_->lineEditAbsoluteMaximum->text().
      toDouble());
}

void CurveAxisScaleConfigWidget::lineEditRelativeMinimumEditingFinished() {
  if (config_)
    config_->setRelativeMinimum(ui_->lineEditRelativeMinimum->text().
      toDouble());
}

void CurveAxisScaleConfigWidget::lineEditRelativeMaximumEditingFinished() {
  if (config_)
    config_->setRelativeMaximum(ui_->lineEditRelativeMaximum->text().
      toDouble());
}

}
