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

#include <ros/package.h>

#include <ui_MultiplotWidget.h>

#include "rqt_multiplot/MultiplotWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MultiplotWidget::MultiplotWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::MultiplotWidget()),
  config_(new MultiplotConfig(this)) {
  ui_->setupUi(this);
  
  ui_->pushButtonRun->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/run.png"))));
  ui_->pushButtonPause->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/pause.png"))));
  ui_->pushButtonClear->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/clear.png"))));
  
  ui_->plotTableWidget->setConfig(config_->getTableConfig());
  
  connect(ui_->spinBoxRows, SIGNAL(valueChanged(int)), this,
    SLOT(spinBoxRowsValueChanged(int)));
  connect(ui_->spinBoxColumns, SIGNAL(valueChanged(int)), this,
    SLOT(spinBoxColumnsValueChanged(int)));

  connect(ui_->pushButtonRun, SIGNAL(clicked()), this,
    SLOT(pushButtonRunClicked()));
  connect(ui_->pushButtonPause, SIGNAL(clicked()), this,
    SLOT(pushButtonPauseClicked()));
  connect(ui_->pushButtonClear, SIGNAL(clicked()), this,
    SLOT(pushButtonClearClicked()));
}

MultiplotWidget::~MultiplotWidget() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

MultiplotConfig* MultiplotWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MultiplotWidget::spinBoxRowsValueChanged(int value) {
  config_->getTableConfig()->setNumRows(value);
}

void MultiplotWidget::spinBoxColumnsValueChanged(int value) {
  config_->getTableConfig()->setNumColumns(value);
}

void MultiplotWidget::pushButtonRunClicked() {
  ui_->plotTableWidget->runPlots();
}

void MultiplotWidget::pushButtonPauseClicked() {
  ui_->plotTableWidget->pausePlots();
}

void MultiplotWidget::pushButtonClearClicked() {
  ui_->plotTableWidget->clearPlots();
}

}
