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

#include <ui_CurveAxisRangeWidget.h>

#include "rqt_multiplot/CurveAxisRangeWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveAxisRangeWidget::CurveAxisRangeWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveAxisRangeWidget()),
  range_(0) {
  ui_->setupUi(this);
  
  ui_->doubleSpinBoxFixedMinimum->setEnabled(false);
  ui_->doubleSpinBoxFixedMaximum->setEnabled(false);
  ui_->doubleSpinBoxWindowSize->setEnabled(false);
  
  connect(ui_->radioButtonFixed, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonFixedToggled(bool)));
  connect(ui_->radioButtonWindow, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonWindowToggled(bool)));
  connect(ui_->radioButtonAuto, SIGNAL(toggled(bool)), this,
    SLOT(radioButtonAutoToggled(bool)));
  
  connect(ui_->doubleSpinBoxFixedMinimum, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxFixedMinimumValueChanged(double)));
  connect(ui_->doubleSpinBoxFixedMaximum, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxFixedMaximumValueChanged(double)));
  connect(ui_->doubleSpinBoxWindowSize, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxWindowSizeValueChanged(double)));
}

CurveAxisRangeWidget::~CurveAxisRangeWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveAxisRangeWidget::setRange(CurveAxisRange* range) {
  if (range != range_) {
    if (range_) {
      disconnect(range_, SIGNAL(typeChanged(int)), this,
        SLOT(rangeTypeChanged(int)));
      disconnect(range_, SIGNAL(fixedMinimumChanged(double)), this,
        SLOT(rangeFixedMinimumChanged(double)));
      disconnect(range_, SIGNAL(fixedMaximumChanged(double)), this,
        SLOT(rangeFixedMaximumChanged(double)));
      disconnect(range_, SIGNAL(windowSizeChanged(double)), this,
        SLOT(rangeWindowSizeChanged(double)));
    }
    
    range_ = range;
    
    if (range) {
      connect(range, SIGNAL(typeChanged(int)), this,
        SLOT(rangeTypeChanged(int)));
      connect(range, SIGNAL(fixedMinimumChanged(double)), this,
        SLOT(rangeFixedMinimumChanged(double)));
      connect(range, SIGNAL(fixedMaximumChanged(double)), this,
        SLOT(rangeFixedMaximumChanged(double)));
      connect(range, SIGNAL(windowSizeChanged(double)), this,
        SLOT(rangeWindowSizeChanged(double)));
      
      rangeTypeChanged(range->getType());
      rangeFixedMinimumChanged(range->getFixedMinimum());
      rangeFixedMaximumChanged(range->getFixedMaximum());
      rangeWindowSizeChanged(range->getWindowSize());
    }
  }
}

CurveAxisRange* CurveAxisRangeWidget::getRange() const {
  return range_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveAxisRangeWidget::rangeTypeChanged(int type) {
  if (type == CurveAxisRange::Fixed)
    ui_->radioButtonFixed->setChecked(true);
  else if (type == CurveAxisRange::Window)
    ui_->radioButtonWindow->setChecked(true);
  else
    ui_->radioButtonAuto->setChecked(true);
}

void CurveAxisRangeWidget::rangeFixedMinimumChanged(double minimum) {
  ui_->doubleSpinBoxFixedMinimum->setValue(minimum);
}

void CurveAxisRangeWidget::rangeFixedMaximumChanged(double maximum) {
  ui_->doubleSpinBoxFixedMaximum->setValue(maximum);
}

void CurveAxisRangeWidget::rangeWindowSizeChanged(double size) {
  ui_->doubleSpinBoxWindowSize->setValue(size);
}

void CurveAxisRangeWidget::radioButtonFixedToggled(bool checked) {
  ui_->doubleSpinBoxFixedMinimum->setEnabled(checked);
  ui_->doubleSpinBoxFixedMaximum->setEnabled(checked);
  
  if (range_ && checked)
    range_->setType(CurveAxisRange::Fixed);
}

void CurveAxisRangeWidget::radioButtonWindowToggled(bool checked) {
  ui_->doubleSpinBoxWindowSize->setEnabled(checked);
  
  if (range_ && checked)
    range_->setType(CurveAxisRange::Window);
}

void CurveAxisRangeWidget::radioButtonAutoToggled(bool checked) {
  if (range_ && checked)
    range_->setType(CurveAxisRange::Auto);
}

void CurveAxisRangeWidget::doubleSpinBoxFixedMinimumValueChanged(double
    value) {
  if (range_)
    range_->setFixedMinimum(value);
}

void CurveAxisRangeWidget::doubleSpinBoxFixedMaximumValueChanged(double
    value) {
  if (range_)
    range_->setFixedMaximum(value);
}

void CurveAxisRangeWidget::doubleSpinBoxWindowSizeValueChanged(double
    value) {
  if (range_)
    range_->setWindowSize(value);
}

}
