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

#include <QColorDialog>

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
  
  ui_->labelBackgroundColor->setAutoFillBackground(true);
  ui_->labelForegroundColor->setAutoFillBackground(true);
  
  ui_->pushButtonRun->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/run.png"))));
  ui_->pushButtonPause->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/pause.png"))));
  ui_->pushButtonClear->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/clear.png"))));
  ui_->pushButtonExport->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/export.png"))));
  
  connect(ui_->spinBoxRows, SIGNAL(valueChanged(int)), this,
    SLOT(spinBoxRowsValueChanged(int)));
  connect(ui_->spinBoxColumns, SIGNAL(valueChanged(int)), this,
    SLOT(spinBoxColumnsValueChanged(int)));

  connect(ui_->checkBoxLinkScale, SIGNAL(stateChanged(int)), this,
    SLOT(checkBoxLinkScaleStateChanged(int)));
  connect(ui_->checkBoxLinkCursor, SIGNAL(stateChanged(int)), this,
    SLOT(checkBoxLinkCursorStateChanged(int)));
  
  connect(ui_->pushButtonRun, SIGNAL(clicked()), this,
    SLOT(pushButtonRunClicked()));
  connect(ui_->pushButtonPause, SIGNAL(clicked()), this,
    SLOT(pushButtonPauseClicked()));
  connect(ui_->pushButtonClear, SIGNAL(clicked()), this,
    SLOT(pushButtonClearClicked()));
  connect(ui_->pushButtonExport, SIGNAL(clicked()), this,
    SLOT(pushButtonExportClicked()));
  
  ui_->labelBackgroundColor->installEventFilter(this);
  ui_->labelForegroundColor->installEventFilter(this);
}

PlotTableConfigWidget::~PlotTableConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotTableConfigWidget::setConfig(PlotTableConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(backgroundColorChanged(const QColor&)), 
        this, SLOT(configBackgroundColorChanged(const QColor&)));
      disconnect(config_, SIGNAL(foregroundColorChanged(const QColor&)), 
        this, SLOT(configForegroundColorChanged(const QColor&)));
      disconnect(config_, SIGNAL(numPlotsChanged(size_t, size_t)), 
        this, SLOT(configNumPlotsChanged(size_t, size_t)));
      disconnect(config_, SIGNAL(linkScaleChanged(bool)), 
        this, SLOT(configLinkScaleChanged(bool)));
      disconnect(config_, SIGNAL(linkCursorChanged(bool)), 
        this, SLOT(configLinkCursorChanged(bool)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(backgroundColorChanged(const QColor&)), 
        this, SLOT(configBackgroundColorChanged(const QColor&)));
      connect(config, SIGNAL(foregroundColorChanged(const QColor&)), 
        this, SLOT(configForegroundColorChanged(const QColor&)));
      connect(config, SIGNAL(numPlotsChanged(size_t, size_t)), 
        this, SLOT(configNumPlotsChanged(size_t, size_t)));
      connect(config, SIGNAL(linkScaleChanged(bool)), 
        this, SLOT(configLinkScaleChanged(bool)));
      connect(config, SIGNAL(linkCursorChanged(bool)), 
        this, SLOT(configLinkCursorChanged(bool)));
      
      configBackgroundColorChanged(config->getBackgroundColor());
      configForegroundColorChanged(config->getForegroundColor());
      configNumPlotsChanged(config->getNumRows(), config->getNumColumns());
      configLinkScaleChanged(config_->isScaleLinked());
      configLinkCursorChanged(config_->isCursorLinked());
    }
  }
}

PlotTableConfig* PlotTableConfigWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool PlotTableConfigWidget::eventFilter(QObject* object, QEvent* event) {
  if (config_) {
    if (((object == ui_->labelBackgroundColor) ||
        (object == ui_->labelForegroundColor)) &&
        (event->type() == QEvent::MouseButtonPress)) {
      QColorDialog dialog(this);
    
      dialog.setCurrentColor((object == ui_->labelBackgroundColor) ?
        config_->getBackgroundColor() : config_->getForegroundColor());
    
      if (dialog.exec() == QDialog::Accepted) {
        if (object == ui_->labelBackgroundColor)
          config_->setBackgroundColor(dialog.currentColor());
        else
          config_->setForegroundColor(dialog.currentColor());
      }
    }
  }
  
  return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotTableConfigWidget::configBackgroundColorChanged(const QColor&
    color) {
  QPalette palette = ui_->labelBackgroundColor->palette();
  palette.setColor(QPalette::Window, color);
  
  ui_->labelBackgroundColor->setPalette(palette);
}

void PlotTableConfigWidget::configForegroundColorChanged(const QColor&
    color) {
  QPalette palette = ui_->labelForegroundColor->palette();
  palette.setColor(QPalette::Window, color);
  
  ui_->labelForegroundColor->setPalette(palette);
}

void PlotTableConfigWidget::configNumPlotsChanged(size_t numRows, size_t
    numColumns) {
  ui_->spinBoxRows->setValue(numRows);
  ui_->spinBoxColumns->setValue(numColumns);
}

void PlotTableConfigWidget::configLinkScaleChanged(bool link) {
  ui_->checkBoxLinkScale->setCheckState(link ? Qt::Checked : Qt::Unchecked);
}

void PlotTableConfigWidget::configLinkCursorChanged(bool link) {
  ui_->checkBoxLinkCursor->setCheckState(link ? Qt::Checked : Qt::Unchecked);
}

void PlotTableConfigWidget::spinBoxRowsValueChanged(int value) {
  if (config_)
    config_->setNumRows(value);
}

void PlotTableConfigWidget::spinBoxColumnsValueChanged(int value) {
  if (config_)
    config_->setNumColumns(value);
}

void PlotTableConfigWidget::checkBoxLinkScaleStateChanged(int state) {
  if (config_)
    config_->setLinkScale(state == Qt::Checked);
}

void PlotTableConfigWidget::checkBoxLinkCursorStateChanged(int state) {
  if (config_)
    config_->setLinkCursor(state == Qt::Checked);
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

void PlotTableConfigWidget::pushButtonExportClicked() {
//   ui_->plotTableWidget->clearPlots();
}

}
