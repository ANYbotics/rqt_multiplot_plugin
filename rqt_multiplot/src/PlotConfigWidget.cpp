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

#include <rqt_multiplot/CurveConfigDialog.h>
#include <rqt_multiplot/CurveConfigWidget.h>
#include <rqt_multiplot/CurveItemWidget.h>

#include <ui_PlotConfigWidget.h>

#include "rqt_multiplot/PlotConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotConfigWidget::PlotConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::PlotConfigWidget()),
  config_(new PlotConfig(this)) {
  ui_->setupUi(this);  
  
  ui_->pushButtonAddCurve->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/add.png"))));
  ui_->pushButtonEditCurve->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/edit.png"))));
  ui_->pushButtonRemoveCurve->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/remove.png"))));
  
  ui_->pushButtonEditCurve->setEnabled(false);
  ui_->pushButtonRemoveCurve->setEnabled(false);
  
  ui_->axesConfigWidget->setConfig(config_->getAxesConfig());
  ui_->legendConfigWidget->setConfig(config_->getLegendConfig());
  
  connect(config_, SIGNAL(titleChanged(const QString&)),
    this, SLOT(configTitleChanged(const QString&)));
  connect(config_, SIGNAL(plotRateChanged(double)),
    this, SLOT(configPlotRateChanged(double)));
    
  connect(ui_->lineEditTitle, SIGNAL(editingFinished()), this,
    SLOT(lineEditTitleEditingFinished()));
  
  connect(ui_->pushButtonAddCurve, SIGNAL(clicked()), this,
    SLOT(pushButtonAddCurveClicked()));
  connect(ui_->pushButtonEditCurve, SIGNAL(clicked()), this,
    SLOT(pushButtonEditCurveClicked()));
  connect(ui_->pushButtonRemoveCurve, SIGNAL(clicked()), this,
    SLOT(pushButtonRemoveCurveClicked()));
  
  connect(ui_->curveListWidget, SIGNAL(itemSelectionChanged()),
    this, SLOT(curveListWidgetItemSelectionChanged()));
  connect(ui_->curveListWidget, SIGNAL(itemDoubleClicked(QListWidgetItem*)),
    this, SLOT(curveListWidgetItemDoubleClicked(QListWidgetItem*)));
  
  connect(ui_->doubleSpinBoxPlotRate, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxPlotRateValueChanged(double)));
  
  configTitleChanged(config_->getTitle());
  configPlotRateChanged(config_->getPlotRate());
}

PlotConfigWidget::~PlotConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotConfigWidget::setConfig(const PlotConfig& config) {
  ui_->curveListWidget->clear();
  
  *config_ = config;
  
  for (size_t index = 0; index < config_->getNumCurves(); ++index) {
    CurveItemWidget* widget = new CurveItemWidget(ui_->curveListWidget);
    widget->setConfig(config_->getCurveConfig(index));
    
    QListWidgetItem* item = new QListWidgetItem(ui_->curveListWidget);
    item->setSizeHint(widget->sizeHint());
    
    ui_->curveListWidget->addItem(item);
    ui_->curveListWidget->setItemWidget(item, widget);
  }
}

const PlotConfig& PlotConfigWidget::getConfig() const {
  return *config_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotConfigWidget::configTitleChanged(const QString& title) {
  ui_->lineEditTitle->setText(title);
}

void PlotConfigWidget::configPlotRateChanged(double rate) {
  ui_->doubleSpinBoxPlotRate->setValue(rate);
}

void PlotConfigWidget::lineEditTitleEditingFinished() {
  config_->setTitle(ui_->lineEditTitle->text());
}

void PlotConfigWidget::pushButtonAddCurveClicked() {
  CurveConfigDialog dialog(this);
  
  dialog.setWindowTitle(config_->getTitle().isEmpty() ?
    "Add Curve to Plot" :
    "Add Curve to \""+config_->getTitle()+"\"");
  dialog.getWidget()->getConfig().getColor()->setAutoColorIndex(
    config_->getNumCurves());

  if (dialog.exec() == QDialog::Accepted) {
    CurveConfig* curveConfig = config_->addCurve();
    *curveConfig = dialog.getWidget()->getConfig();
    
    CurveItemWidget* widget = new CurveItemWidget(ui_->curveListWidget);
    widget->setConfig(curveConfig);
    
    QListWidgetItem* item = new QListWidgetItem(ui_->curveListWidget);
    item->setSizeHint(widget->sizeHint());
    
    ui_->curveListWidget->addItem(item);
    ui_->curveListWidget->setItemWidget(item, widget);
  }
}

void PlotConfigWidget::pushButtonEditCurveClicked() {
  QListWidgetItem* item = ui_->curveListWidget->currentItem();
  
  if (item) {
    CurveItemWidget* widget = static_cast<CurveItemWidget*>(
      ui_->curveListWidget->itemWidget(item));
    CurveConfig* curveConfig = widget->getConfig();
    
    CurveConfigDialog dialog(this);
    
    dialog.setWindowTitle(curveConfig->getTitle().isEmpty() ?
      "Edit Curve" :
      "Edit \""+curveConfig->getTitle()+"\"");
    dialog.getWidget()->setConfig(*curveConfig);
    
    if (dialog.exec() == QDialog::Accepted) {
      *curveConfig = dialog.getWidget()->getConfig();
    }
  }
}

void PlotConfigWidget::pushButtonRemoveCurveClicked() {
  QListWidgetItem* item = ui_->curveListWidget->currentItem();
  
  if (item) {
    CurveItemWidget* widget = static_cast<CurveItemWidget*>(
      ui_->curveListWidget->itemWidget(item));
    CurveConfig* curveConfig = widget->getConfig();
    
    delete item;
    
    config_->removeCurve(curveConfig);
  }
}

void PlotConfigWidget::curveListWidgetItemSelectionChanged() {
  QListWidgetItem* currentItem = ui_->curveListWidget->currentItem();
  
  ui_->pushButtonEditCurve->setEnabled(currentItem);
  ui_->pushButtonRemoveCurve->setEnabled(currentItem);
}

void PlotConfigWidget::curveListWidgetItemDoubleClicked(QListWidgetItem*
    item) {
  pushButtonEditCurveClicked();
}

void PlotConfigWidget::doubleSpinBoxPlotRateValueChanged(double value) {
  config_->setPlotRate(value);
}

}
