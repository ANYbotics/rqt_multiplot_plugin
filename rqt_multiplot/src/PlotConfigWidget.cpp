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
  
  ui_->pushButtonAdd->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/add.png"))));
  ui_->pushButtonEdit->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/edit.png"))));
  ui_->pushButtonRemove->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/remove.png"))));
  
  ui_->pushButtonEdit->setEnabled(false);
  ui_->pushButtonRemove->setEnabled(false);
  
  connect(config_, SIGNAL(titleChanged(const QString&)),
    this, SLOT(configTitleChanged(const QString&)));
    
  connect(ui_->lineEditTitle, SIGNAL(editingFinished()), this,
    SLOT(lineEditTitleEditingFinished()));
  
  connect(ui_->pushButtonAdd, SIGNAL(clicked()), this,
    SLOT(pushButtonAddClicked()));
  connect(ui_->pushButtonEdit, SIGNAL(clicked()), this,
    SLOT(pushButtonEditClicked()));
  connect(ui_->pushButtonRemove, SIGNAL(clicked()), this,
    SLOT(pushButtonRemoveClicked()));
  
  connect(ui_->listWidgetCurves, SIGNAL(itemSelectionChanged()),
    this, SLOT(listWidgetCurvesItemSelectionChanged()));
  connect(ui_->listWidgetCurves, SIGNAL(itemDoubleClicked(QListWidgetItem*)),
    this, SLOT(listWidgetCurvesItemDoubleClicked(QListWidgetItem*)));
  
  configTitleChanged(config_->getTitle());
}

PlotConfigWidget::~PlotConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotConfigWidget::setConfig(const PlotConfig& config) {
  ui_->listWidgetCurves->clear();
  
  *config_ = config;
  
  for (size_t index = 0; index < config_->getNumCurves(); ++index) {
    CurveItemWidget* widget = new CurveItemWidget(ui_->listWidgetCurves);
    widget->setConfig(config_->getCurveConfig(index));
    
    QListWidgetItem* item = new QListWidgetItem(ui_->listWidgetCurves);
    item->setSizeHint(widget->sizeHint());
    
    ui_->listWidgetCurves->addItem(item);
    ui_->listWidgetCurves->setItemWidget(item, widget);
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

void PlotConfigWidget::lineEditTitleEditingFinished() {
  if (config_)
    config_->setTitle(ui_->lineEditTitle->text());
}

void PlotConfigWidget::pushButtonAddClicked() {
  CurveConfigDialog dialog(this);
  
  dialog.setWindowTitle(config_->getTitle().isEmpty() ?
    "Add Curve to Plot" :
    "Add Curve to \""+config_->getTitle()+"\"");
  dialog.getWidget()->getConfig().getColor()->setAutoColorIndex(
    config_->getNumCurves());

  if (dialog.exec() == QDialog::Accepted) {
    CurveConfig* curveConfig = config_->addCurve();
    *curveConfig = dialog.getWidget()->getConfig();
    
    CurveItemWidget* widget = new CurveItemWidget(ui_->listWidgetCurves);
    widget->setConfig(curveConfig);
    
    QListWidgetItem* item = new QListWidgetItem(ui_->listWidgetCurves);
    item->setSizeHint(widget->sizeHint());
    
    ui_->listWidgetCurves->addItem(item);
    ui_->listWidgetCurves->setItemWidget(item, widget);
  }
}

void PlotConfigWidget::pushButtonEditClicked() {
  QListWidgetItem* item = ui_->listWidgetCurves->currentItem();
  
  if (item) {
    CurveItemWidget* widget = static_cast<CurveItemWidget*>(
      ui_->listWidgetCurves->itemWidget(item));
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

void PlotConfigWidget::pushButtonRemoveClicked() {
  QListWidgetItem* item = ui_->listWidgetCurves->currentItem();
  
  if (item) {
    CurveItemWidget* widget = static_cast<CurveItemWidget*>(
      ui_->listWidgetCurves->itemWidget(item));
    CurveConfig* curveConfig = widget->getConfig();
    
    delete item;
    
    config_->removeCurve(curveConfig);
  }
}

void PlotConfigWidget::listWidgetCurvesItemSelectionChanged() {
  QListWidgetItem* currentItem = ui_->listWidgetCurves->currentItem();
  
  ui_->pushButtonEdit->setEnabled(currentItem);
  ui_->pushButtonRemove->setEnabled(currentItem);
}

void PlotConfigWidget::listWidgetCurvesItemDoubleClicked(QListWidgetItem*
    item) {
  pushButtonEditClicked();
}

}
