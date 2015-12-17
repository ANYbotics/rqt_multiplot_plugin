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

#include <ui_CurveStyleConfigWidget.h>

#include "rqt_multiplot/CurveStyleConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveStyleConfigWidget::CurveStyleConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveStyleConfigWidget()),
  buttonGroupSticksOrientation_(new QButtonGroup(this)),
  config_(0) {
  ui_->setupUi(this);
  
  ui_->lineEditSticksBaseline->setValidator(new QDoubleValidator(
    ui_->lineEditSticksBaseline));
  
  ui_->checkBoxLinesInterpolate->setEnabled(false);
  ui_->radioButtonSticksOrientationHorizontal->setEnabled(false);
  ui_->radioButtonSticksOrientationVertical->setEnabled(false);
  ui_->labelSticksBaseline->setEnabled(false);
  ui_->lineEditSticksBaseline->setEnabled(false);
  ui_->checkBoxStepsInvert->setEnabled(false);
  
  buttonGroupSticksOrientation_->addButton(
    ui_->radioButtonSticksOrientationHorizontal);
  buttonGroupSticksOrientation_->addButton(
    ui_->radioButtonSticksOrientationVertical);
  
  connect(ui_->radioButtonLines, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonLinesToggled(bool)));
  connect(ui_->radioButtonSticks, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonSticksToggled(bool)));
  connect(ui_->radioButtonSteps, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonStepsToggled(bool)));
  connect(ui_->radioButtonPoints, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonPointsToggled(bool)));
  
  connect(ui_->checkBoxLinesInterpolate, SIGNAL(stateChanged(int)),
    this, SLOT(checkBoxLinesInterpolateStateChanged(int)));
  connect(ui_->radioButtonSticksOrientationHorizontal, SIGNAL(toggled(bool)),
    this, SLOT(radioButtonSticksOrientationHorizontalToggled(bool)));
  connect(ui_->radioButtonSticksOrientationVertical, SIGNAL(toggled(bool)),
    this, SLOT(radioButtonSticksOrientationVerticalToggled(bool)));
  connect(ui_->lineEditSticksBaseline, SIGNAL(editingFinished()),
    this, SLOT(lineEditSticksBaselineEditingFinished()));  
  connect(ui_->checkBoxStepsInvert, SIGNAL(stateChanged(int)),
    this, SLOT(checkBoxStepsInvertStateChanged(int)));
  
  connect(ui_->spinBoxPenWidth, SIGNAL(valueChanged(int)),
    this, SLOT(spinBoxPenWidthValueChanged(int)));
  connect(ui_->comboBoxPenStyle, SIGNAL(currentStyleChanged(int)),
    this, SLOT(comboBoxPenStyleCurrentStyleChanged(int)));
  connect(ui_->checkBoxRenderAntialias, SIGNAL(stateChanged(int)),
    this, SLOT(checkBoxRenderAntialiasStateChanged(int)));
}

CurveStyleConfigWidget::~CurveStyleConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveStyleConfigWidget::setConfig(CurveStyleConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(typeChanged(int)), this,
        SLOT(styleTypeChanged(int)));
      
      disconnect(config_, SIGNAL(linesInterpolateChanged(bool)),
        this, SLOT(configLinesInterpolateChanged(bool)));
      disconnect(config_, SIGNAL(sticksOrientationChanged(int)),
        this, SLOT(configSticksOrientationChanged(int)));
      disconnect(config_, SIGNAL(sticksBaselineChanged(double)),
        this, SLOT(configSticksBaselineChanged(double)));
      disconnect(config_, SIGNAL(stepsInvertChanged(bool)),
        this, SLOT(configStepsInvertChanged(bool)));
      
      disconnect(config_, SIGNAL(penWidthChanged(size_t)),
        this, SLOT(configPenWidthChanged(size_t)));
      disconnect(config_, SIGNAL(penStyleChanged(int)),
        this, SLOT(configPenStyleChanged(int)));
      disconnect(config_, SIGNAL(renderAntialiasChanged(bool)),
        this, SLOT(configRenderAntialiasChanged(bool)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(typeChanged(int)), this,
        SLOT(configTypeChanged(int)));
      
      connect(config, SIGNAL(linesInterpolateChanged(bool)),
        this, SLOT(configLinesInterpolateChanged(bool)));
      connect(config, SIGNAL(sticksOrientationChanged(int)),
        this, SLOT(configSticksOrientationChanged(int)));
      connect(config, SIGNAL(sticksBaselineChanged(double)),
        this, SLOT(configSticksBaselineChanged(double)));
      connect(config, SIGNAL(stepsInvertChanged(bool)),
        this, SLOT(configStepsInvertChanged(bool)));
      
      connect(config, SIGNAL(penWidthChanged(size_t)),
        this, SLOT(configPenWidthChanged(size_t)));
      connect(config, SIGNAL(penStyleChanged(int)),
        this, SLOT(configPenStyleChanged(int)));
      connect(config, SIGNAL(renderAntialiasChanged(bool)),
        this, SLOT(configRenderAntialiasChanged(bool)));
      
      configTypeChanged(config->getType());
      
      configLinesInterpolateChanged(config->areLinesInterpolated());
      configSticksOrientationChanged(config->getSticksOrientation());
      configSticksBaselineChanged(config->getSticksBaseline());
      configStepsInvertChanged(config->areStepsInverted());
      
      configPenWidthChanged(config->getPenWidth());
      configPenStyleChanged(config->getPenStyle());
      configRenderAntialiasChanged(config->isRenderAntialiased());
    }
  }
}

CurveStyleConfig* CurveStyleConfigWidget::getConfig() const {
  return config_;
}
 
/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveStyleConfigWidget::configTypeChanged(int type) {
  if (type == CurveStyleConfig::Sticks)
    ui_->radioButtonSticks->setChecked(true);
  else if (type == CurveStyleConfig::Steps)
    ui_->radioButtonSteps->setChecked(true);
  else if (type == CurveStyleConfig::Points)
    ui_->radioButtonPoints->setChecked(true);
  else
    ui_->radioButtonLines->setChecked(true);
}

void CurveStyleConfigWidget::configLinesInterpolateChanged(bool interpolate) {
  ui_->checkBoxLinesInterpolate->setCheckState(interpolate ? Qt::Checked :
    Qt::Unchecked);
}

void CurveStyleConfigWidget::configSticksOrientationChanged(int orientation) {
  ui_->radioButtonSticksOrientationHorizontal->setChecked(
    orientation == Qt::Horizontal);
  ui_->radioButtonSticksOrientationVertical->setChecked(
    orientation == Qt::Vertical);
}

void CurveStyleConfigWidget::configSticksBaselineChanged(double baseline) {
  ui_->lineEditSticksBaseline->setText(QString::number(baseline));
}

void CurveStyleConfigWidget::configStepsInvertChanged(bool invert) {
  ui_->checkBoxStepsInvert->setCheckState(invert ? Qt::Checked :
    Qt::Unchecked);
}

void CurveStyleConfigWidget::configPenWidthChanged(size_t width) {
  ui_->spinBoxPenWidth->setValue(width);
}

void CurveStyleConfigWidget::configPenStyleChanged(int style) {
  ui_->comboBoxPenStyle->setCurrentStyle(static_cast<Qt::PenStyle>(style));
}

void CurveStyleConfigWidget::configRenderAntialiasChanged(bool antialias) {
  ui_->checkBoxRenderAntialias->setCheckState(antialias ? Qt::Checked :
    Qt::Unchecked);
}

void CurveStyleConfigWidget::radioButtonLinesToggled(bool checked) {
  ui_->checkBoxLinesInterpolate->setEnabled(checked);
  
  if (config_ && checked)
    config_->setType(CurveStyleConfig::Lines);
}

void CurveStyleConfigWidget::radioButtonSticksToggled(bool checked) {
  ui_->radioButtonSticksOrientationHorizontal->setEnabled(checked);
  ui_->radioButtonSticksOrientationVertical->setEnabled(checked);
  ui_->labelSticksBaseline->setEnabled(checked);
  ui_->lineEditSticksBaseline->setEnabled(checked);
  
  if (config_ && checked)
    config_->setType(CurveStyleConfig::Sticks);
}

void CurveStyleConfigWidget::radioButtonStepsToggled(bool checked) {
  ui_->checkBoxStepsInvert->setEnabled(checked);
  
  if (config_ && checked)
    config_->setType(CurveStyleConfig::Steps);
}

void CurveStyleConfigWidget::radioButtonPointsToggled(bool checked) {
  if (config_ && checked)
    config_->setType(CurveStyleConfig::Points);
}

void CurveStyleConfigWidget::checkBoxLinesInterpolateStateChanged(int state) {
  if (config_)
    config_->setLinesInterpolate(state == Qt::Checked);
}

void CurveStyleConfigWidget::radioButtonSticksOrientationHorizontalToggled(
    bool checked) {
  if (config_ && checked)
    config_->setSticksOrientation(Qt::Horizontal);
}

void CurveStyleConfigWidget::radioButtonSticksOrientationVerticalToggled(
    bool checked) {
  if (config_ && checked)
    config_->setSticksOrientation(Qt::Vertical);
}

void CurveStyleConfigWidget::lineEditSticksBaselineEditingFinished() {
  if (config_)
    config_->setSticksBaseline(ui_->lineEditSticksBaseline->text().
      toDouble());
}

void CurveStyleConfigWidget::checkBoxStepsInvertStateChanged(int state) {
  if (config_)
    config_->setStepsInvert(state == Qt::Checked);
}

void CurveStyleConfigWidget::spinBoxPenWidthValueChanged(int value) {
  if (config_)
    config_->setPenWidth(value);
}

void CurveStyleConfigWidget::comboBoxPenStyleCurrentStyleChanged(int style) {
  if (config_)
    config_->setPenStyle(static_cast<Qt::PenStyle>(style));
}

void CurveStyleConfigWidget::checkBoxRenderAntialiasStateChanged(int state) {
  if (config_)
    config_->setRenderAntialias(state == Qt::Checked);
}

}
