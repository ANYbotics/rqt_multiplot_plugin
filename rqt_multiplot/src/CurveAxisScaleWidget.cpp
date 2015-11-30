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

#include <ui_CurveAxisScaleWidget.h>

#include "rqt_multiplot/CurveAxisScaleWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveAxisScaleWidget::CurveAxisScaleWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveAxisScaleWidget()),
  scale_(0) {
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

CurveAxisScaleWidget::~CurveAxisScaleWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveAxisScaleWidget::setScale(CurveAxisScale* scale) {
  if (scale != scale_) {
    if (scale_) {
      disconnect(scale_, SIGNAL(typeChanged(int)), this,
        SLOT(scaleTypeChanged(int)));
      disconnect(scale_, SIGNAL(absoluteMinimumChanged(double)), this,
        SLOT(scaleAbsoluteMinimumChanged(double)));
      disconnect(scale_, SIGNAL(absoluteMaximumChanged(double)), this,
        SLOT(scaleAbsoluteMaximumChanged(double)));
      disconnect(scale_, SIGNAL(relativeMinimumChanged(double)), this,
        SLOT(scaleRelativeMinimumChanged(double)));
      disconnect(scale_, SIGNAL(relativeMaximumChanged(double)), this,
        SLOT(scaleRelativeMaximumChanged(double)));
    }
    
    scale_ = scale;
    
    if (scale) {
      connect(scale, SIGNAL(typeChanged(int)), this,
        SLOT(scaleTypeChanged(int)));
      connect(scale, SIGNAL(absoluteMinimumChanged(double)), this,
        SLOT(scaleAbsoluteMinimumChanged(double)));
      connect(scale, SIGNAL(absoluteMaximumChanged(double)), this,
        SLOT(scaleAbsoluteMaximumChanged(double)));
      connect(scale, SIGNAL(relativeMinimumChanged(double)), this,
        SLOT(scaleRelativeMinimumChanged(double)));
      connect(scale, SIGNAL(relativeMaximumChanged(double)), this,
        SLOT(scaleRelativeMaximumChanged(double)));
      
      scaleTypeChanged(scale->getType());
      scaleAbsoluteMinimumChanged(scale->getAbsoluteMinimum());
      scaleAbsoluteMaximumChanged(scale->getAbsoluteMaximum());
      scaleRelativeMinimumChanged(scale->getRelativeMinimum());
      scaleRelativeMaximumChanged(scale->getRelativeMaximum());
    }
  }
}

CurveAxisScale* CurveAxisScaleWidget::getScale() const {
  return scale_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveAxisScaleWidget::scaleTypeChanged(int type) {
  if (type == CurveAxisScale::Absolute)
    ui_->radioButtonAbsolute->setChecked(true);
  else if (type == CurveAxisScale::Relative)
    ui_->radioButtonRelative->setChecked(true);
  else
    ui_->radioButtonAuto->setChecked(true);
}

void CurveAxisScaleWidget::scaleAbsoluteMinimumChanged(double minimum) {
  ui_->lineEditAbsoluteMinimum->setText(QString::number(minimum));
}

void CurveAxisScaleWidget::scaleAbsoluteMaximumChanged(double maximum) {
  ui_->lineEditAbsoluteMaximum->setText(QString::number(maximum));
}

void CurveAxisScaleWidget::scaleRelativeMinimumChanged(double minimum) {
  ui_->lineEditRelativeMinimum->setText(QString::number(minimum));
}

void CurveAxisScaleWidget::scaleRelativeMaximumChanged(double maximum) {
  ui_->lineEditRelativeMaximum->setText(QString::number(maximum));
}

void CurveAxisScaleWidget::radioButtonAbsoluteToggled(bool checked) {
  ui_->lineEditAbsoluteMinimum->setEnabled(checked);
  ui_->lineEditAbsoluteMaximum->setEnabled(checked);
  
  if (scale_ && checked)
    scale_->setType(CurveAxisScale::Absolute);
}

void CurveAxisScaleWidget::radioButtonRelativeToggled(bool checked) {
  ui_->lineEditRelativeMinimum->setEnabled(checked);
  ui_->lineEditRelativeMaximum->setEnabled(checked);
  
  if (scale_ && checked)
    scale_->setType(CurveAxisScale::Relative);
}

void CurveAxisScaleWidget::radioButtonAutoToggled(bool checked) {
  if (scale_ && checked)
    scale_->setType(CurveAxisScale::Auto);
}

void CurveAxisScaleWidget::lineEditAbsoluteMinimumEditingFinished() {
  if (scale_)
    scale_->setAbsoluteMinimum(ui_->lineEditAbsoluteMinimum->text().
      toDouble());
}

void CurveAxisScaleWidget::lineEditAbsoluteMaximumEditingFinished() {
  if (scale_)
    scale_->setAbsoluteMaximum(ui_->lineEditAbsoluteMaximum->text().
      toDouble());
}

void CurveAxisScaleWidget::lineEditRelativeMinimumEditingFinished() {
  if (scale_)
    scale_->setRelativeMinimum(ui_->lineEditRelativeMinimum->text().
      toDouble());
}

void CurveAxisScaleWidget::lineEditRelativeMaximumEditingFinished() {
  if (scale_)
    scale_->setRelativeMaximum(ui_->lineEditRelativeMaximum->text().
      toDouble());
}

}
