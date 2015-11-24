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

#include <ui_PlotTableConfigWidget.h>

#include "rqt_multiplot/PlotTableConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotTableConfigWidget::PlotTableConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::PlotTableConfigWidget()),
  config_(0) {
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

PlotTableConfigWidget::~PlotTableConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotTableConfigWidget::setConfig(PlotTableConfig* config) {
  config_ = config;
}

PlotTableConfig* PlotTableConfigWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotTableConfigWidget::spinBoxRowsValueChanged(int value) {
  if (config_)
    config_->setNumRows(value);
}

void PlotTableConfigWidget::spinBoxColumnsValueChanged(int value) {
  if (config_)
    config_->setNumColumns(value);
}

void PlotTableConfigWidget::pushButtonRunClicked() {
//   ui_->plotTableWidget->runPlots();
}

void PlotTableConfigWidget::pushButtonPauseClicked() {
//   ui_->plotTableWidget->pausePlots();
}

void PlotTableConfigWidget::pushButtonClearClicked() {
//   ui_->plotTableWidget->clearPlots();
}

}
