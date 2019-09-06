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

#include <QApplication>
#include <QClipboard>
#include <QKeyEvent>
#include <QLayout>
#include <QMimeData>

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
  ui_->pushButtonRemoveCurves->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/remove.png"))));

  ui_->pushButtonEditCurve->setEnabled(false);
  ui_->pushButtonRemoveCurves->setEnabled(false);
  
  ui_->pushButtonCopyCurves->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/copy.png"))));
  ui_->pushButtonPasteCurves->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/paste.png"))));
  
  ui_->pushButtonCopyCurves->setEnabled(false);
  ui_->pushButtonPasteCurves->setEnabled(false);
  
  ui_->curveListWidget->installEventFilter(this);
  
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
  connect(ui_->pushButtonRemoveCurves, SIGNAL(clicked()), this,
    SLOT(pushButtonRemoveCurvesClicked()));
  
  connect(ui_->pushButtonCopyCurves, SIGNAL(clicked()), this,
    SLOT(pushButtonCopyCurvesClicked()));
  connect(ui_->pushButtonPasteCurves, SIGNAL(clicked()), this,
    SLOT(pushButtonPasteCurvesClicked()));
  
  connect(ui_->curveListWidget, SIGNAL(itemSelectionChanged()),
    this, SLOT(curveListWidgetItemSelectionChanged()));
  connect(ui_->curveListWidget, SIGNAL(itemDoubleClicked(QListWidgetItem*)),
    this, SLOT(curveListWidgetItemDoubleClicked(QListWidgetItem*)));
  
  connect(ui_->doubleSpinBoxPlotRate, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxPlotRateValueChanged(double)));
  
  connect(QApplication::clipboard(), SIGNAL(dataChanged()),
    this, SLOT(clipboardDataChanged()));
  
  configTitleChanged(config_->getTitle());
  configPlotRateChanged(config_->getPlotRate());
  
  clipboardDataChanged();
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
  
  for (size_t index = 0; index < config_->getNumCurves(); ++index)
    ui_->curveListWidget->addCurve(config_->getCurveConfig(index));
}

const PlotConfig& PlotConfigWidget::getConfig() const {
  return *config_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotConfigWidget::copySelectedCurves() {
  QList<QListWidgetItem*> items = ui_->curveListWidget->selectedItems();
  
  if (!items.isEmpty()) {
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    
    stream << (quint64)items.count();
    
    for (QList<QListWidgetItem*>::const_iterator it = items.begin();
        it != items.end(); ++it) {
      CurveItemWidget* itemWidget = static_cast<CurveItemWidget*>(
        ui_->curveListWidget->itemWidget(*it));
      
      stream << *itemWidget->getConfig();
    }
    
    QMimeData* mimeData = new QMimeData();
    mimeData->setData(CurveConfig::MimeType+"-list", data);
    
    QApplication::clipboard()->setMimeData(mimeData);
  }
}

void PlotConfigWidget::pasteCurves() {
  const QMimeData* mimeData = QApplication::clipboard()->mimeData();
  
  if (mimeData && mimeData->hasFormat(CurveConfig::MimeType+"-list")) {
    QByteArray data = mimeData->data(CurveConfig::MimeType+"-list");
    QDataStream stream(&data, QIODevice::ReadOnly);
    
    quint64 numCurves;
    stream >> numCurves;
    
    for (size_t index = 0; index < numCurves; ++index) {
      CurveConfig* curveConfig = config_->addCurve();
      stream >> *curveConfig;

      while (config_->findCurves(curveConfig->getTitle()).count() > 1)
        curveConfig->setTitle("Copy of "+curveConfig->getTitle());
      
      ui_->curveListWidget->addCurve(curveConfig);
    }
  }
}

bool PlotConfigWidget::eventFilter(QObject* object, QEvent* event) {
  if (object == ui_->curveListWidget) {
    if (event->type() == QEvent::KeyPress) {
      QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
      
      if (keyEvent->modifiers() == Qt::ControlModifier) {
        if (keyEvent->key() == Qt::Key_C)
          copySelectedCurves();
        else if  (keyEvent->key() == Qt::Key_V)
          pasteCurves();
      }
    }
  }
  
  return false;
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
  dialog.getWidget()->getConfig().getColorConfig()->setAutoColorIndex(
    config_->getNumCurves());

  if (dialog.exec() == QDialog::Accepted) {
    CurveConfig* curveConfig = config_->addCurve();
    *curveConfig = dialog.getWidget()->getConfig();
    
    ui_->curveListWidget->addCurve(curveConfig);
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
    
    if (dialog.exec() == QDialog::Accepted)
      *curveConfig = dialog.getWidget()->getConfig();
  }
}

void PlotConfigWidget::pushButtonRemoveCurvesClicked() {
  QList<QListWidgetItem*> items = ui_->curveListWidget->selectedItems();
    
  for (QList<QListWidgetItem*>::iterator it = items.begin();
      it != items.end(); ++it) {
    CurveItemWidget* widget = static_cast<CurveItemWidget*>(
      ui_->curveListWidget->itemWidget(*it));
    CurveConfig* curveConfig = widget->getConfig();
    
    delete *it;
    
    config_->removeCurve(curveConfig);
  }
}

void PlotConfigWidget::pushButtonCopyCurvesClicked() {
  copySelectedCurves();
}

void PlotConfigWidget::pushButtonPasteCurvesClicked() {
  pasteCurves();
}

void PlotConfigWidget::curveListWidgetItemSelectionChanged() {
  QList<QListWidgetItem*> items = ui_->curveListWidget->selectedItems();
  
  ui_->pushButtonEditCurve->setEnabled(items.count() == 1);
  ui_->pushButtonRemoveCurves->setEnabled(!items.isEmpty());
  
  ui_->pushButtonCopyCurves->setEnabled(!items.isEmpty());
}

void PlotConfigWidget::curveListWidgetItemDoubleClicked(QListWidgetItem*
    item) {
  pushButtonEditCurveClicked();
}

void PlotConfigWidget::doubleSpinBoxPlotRateValueChanged(double value) {
  config_->setPlotRate(value);
}

void PlotConfigWidget::clipboardDataChanged() {
  const QMimeData* mimeData = QApplication::clipboard()->mimeData();
  
  ui_->pushButtonPasteCurves->setEnabled(mimeData && mimeData->hasFormat(
    CurveConfig::MimeType+"-list"));
}

}
