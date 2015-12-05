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

#include <ui_CurveStyleWidget.h>

#include "rqt_multiplot/CurveStyleWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveStyleWidget::CurveStyleWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveStyleWidget()),
  buttonGroupSticksOrientation_(new QButtonGroup(this)),
  style_(0) {
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

CurveStyleWidget::~CurveStyleWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveStyleWidget::setStyle(CurveStyle* style) {
  if (style != style_) {
    if (style_) {
      disconnect(style_, SIGNAL(typeChanged(int)), this,
        SLOT(styleTypeChanged(int)));
      
      disconnect(style_, SIGNAL(linesInterpolateChanged(bool)),
        this, SLOT(styleLinesInterpolateChanged(bool)));
      disconnect(style_, SIGNAL(sticksOrientationChanged(int)),
        this, SLOT(styleSticksOrientationChanged(int)));
      disconnect(style_, SIGNAL(sticksBaselineChanged(double)),
        this, SLOT(styleSticksBaselineChanged(double)));
      disconnect(style_, SIGNAL(stepsInvertChanged(bool)),
        this, SLOT(styleStepsInvertChanged(bool)));
      
      disconnect(style_, SIGNAL(penWidthChanged(size_t)),
        this, SLOT(stylePenWidthChanged(size_t)));
      disconnect(style_, SIGNAL(penStyleChanged(int)),
        this, SLOT(stylePenStyleChanged(int)));
      disconnect(style_, SIGNAL(renderAntialiasChanged(bool)),
        this, SLOT(styleRenderAntialiasChanged(bool)));
    }
    
    style_ = style;
    
    if (style) {
      connect(style, SIGNAL(typeChanged(int)), this,
        SLOT(styleTypeChanged(int)));
      
      connect(style, SIGNAL(linesInterpolateChanged(bool)),
        this, SLOT(styleLinesInterpolateChanged(bool)));
      connect(style, SIGNAL(sticksOrientationChanged(int)),
        this, SLOT(styleSticksOrientationChanged(int)));
      connect(style, SIGNAL(sticksBaselineChanged(double)),
        this, SLOT(styleSticksBaselineChanged(double)));
      connect(style, SIGNAL(stepsInvertChanged(bool)),
        this, SLOT(styleStepsInvertChanged(bool)));
      
      connect(style, SIGNAL(penWidthChanged(size_t)),
        this, SLOT(stylePenWidthChanged(size_t)));
      connect(style, SIGNAL(penStyleChanged(int)),
        this, SLOT(stylePenStyleChanged(int)));
      connect(style, SIGNAL(renderAntialiasChanged(bool)),
        this, SLOT(styleRenderAntialiasChanged(bool)));
      
      styleTypeChanged(style->getType());
      
      styleLinesInterpolateChanged(style->areLinesInterpolated());
      styleSticksOrientationChanged(style->getSticksOrientation());
      styleSticksBaselineChanged(style->getSticksBaseline());
      styleStepsInvertChanged(style->areStepsInverted());
      
      stylePenWidthChanged(style->getPenWidth());
      stylePenStyleChanged(style->getPenStyle());
      styleRenderAntialiasChanged(style->isRenderAntialiased());
    }
  }
}

CurveStyle* CurveStyleWidget::getStyle() const {
  return style_;
}
 
/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveStyleWidget::styleTypeChanged(int type) {
  if (type == CurveStyle::Sticks)
    ui_->radioButtonSticks->setChecked(true);
  else if (type == CurveStyle::Steps)
    ui_->radioButtonSteps->setChecked(true);
  else if (type == CurveStyle::Points)
    ui_->radioButtonPoints->setChecked(true);
  else
    ui_->radioButtonLines->setChecked(true);
}

void CurveStyleWidget::styleLinesInterpolateChanged(bool interpolate) {
  ui_->checkBoxLinesInterpolate->setCheckState(interpolate ? Qt::Checked :
    Qt::Unchecked);
}

void CurveStyleWidget::styleSticksOrientationChanged(int orientation) {
  ui_->radioButtonSticksOrientationHorizontal->setChecked(
    orientation == Qt::Horizontal);
  ui_->radioButtonSticksOrientationVertical->setChecked(
    orientation == Qt::Vertical);
}

void CurveStyleWidget::styleSticksBaselineChanged(double baseline) {
  ui_->lineEditSticksBaseline->setText(QString::number(baseline));
}

void CurveStyleWidget::styleStepsInvertChanged(bool invert) {
  ui_->checkBoxStepsInvert->setCheckState(invert ? Qt::Checked :
    Qt::Unchecked);
}

void CurveStyleWidget::stylePenWidthChanged(size_t width) {
  ui_->spinBoxPenWidth->setValue(width);
}

void CurveStyleWidget::stylePenStyleChanged(int style) {
  ui_->comboBoxPenStyle->setCurrentStyle(static_cast<Qt::PenStyle>(style));
}

void CurveStyleWidget::styleRenderAntialiasChanged(bool antialias) {
  ui_->checkBoxRenderAntialias->setCheckState(antialias ? Qt::Checked :
    Qt::Unchecked);
}

void CurveStyleWidget::radioButtonLinesToggled(bool checked) {
  ui_->checkBoxLinesInterpolate->setEnabled(checked);
  
  if (style_ && checked)
    style_->setType(CurveStyle::Lines);
}

void CurveStyleWidget::radioButtonSticksToggled(bool checked) {
  ui_->radioButtonSticksOrientationHorizontal->setEnabled(checked);
  ui_->radioButtonSticksOrientationVertical->setEnabled(checked);
  ui_->labelSticksBaseline->setEnabled(checked);
  ui_->lineEditSticksBaseline->setEnabled(checked);
  
  if (style_ && checked)
    style_->setType(CurveStyle::Sticks);
}

void CurveStyleWidget::radioButtonStepsToggled(bool checked) {
  ui_->checkBoxStepsInvert->setEnabled(checked);
  
  if (style_ && checked)
    style_->setType(CurveStyle::Steps);
}

void CurveStyleWidget::radioButtonPointsToggled(bool checked) {
  if (style_ && checked)
    style_->setType(CurveStyle::Points);
}

void CurveStyleWidget::checkBoxLinesInterpolateStateChanged(int state) {
  if (style_)
    style_->setLinesInterpolate(state == Qt::Checked);
}

void CurveStyleWidget::radioButtonSticksOrientationHorizontalToggled(bool
    checked) {
  if (style_ && checked)
    style_->setSticksOrientation(Qt::Horizontal);
}

void CurveStyleWidget::radioButtonSticksOrientationVerticalToggled(bool
    checked) {
  if (style_ && checked)
    style_->setSticksOrientation(Qt::Vertical);
}

void CurveStyleWidget::lineEditSticksBaselineEditingFinished() {
  if (style_)
    style_->setSticksBaseline(ui_->lineEditSticksBaseline->text().
      toDouble());
}

void CurveStyleWidget::checkBoxStepsInvertStateChanged(int state) {
  if (style_)
    style_->setStepsInvert(state == Qt::Checked);
}

void CurveStyleWidget::spinBoxPenWidthValueChanged(int value) {
  if (style_)
    style_->setPenWidth(value);
}

void CurveStyleWidget::comboBoxPenStyleCurrentStyleChanged(int style) {
  if (style_)
    style_->setPenStyle(static_cast<Qt::PenStyle>(style));
}

void CurveStyleWidget::checkBoxRenderAntialiasStateChanged(int state) {
  if (style_)
    style_->setRenderAntialias(state == Qt::Checked);
}

}
