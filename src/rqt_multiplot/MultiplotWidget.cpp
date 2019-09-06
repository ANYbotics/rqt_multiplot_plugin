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
  config_(new MultiplotConfig(this)),
  messageTypeRegistry_(new MessageTypeRegistry(this)),
  packageRegistry_(new PackageRegistry(this)) {
  ui_->setupUi(this);

  ui_->configWidget->setConfig(config_);
  ui_->plotTableConfigWidget->setConfig(config_->getTableConfig());
  ui_->plotTableConfigWidget->setPlotTable(ui_->plotTableWidget);
  ui_->plotTableWidget->setConfig(config_->getTableConfig());

  connect(ui_->configWidget, SIGNAL(currentConfigModifiedChanged(bool)),
    this, SLOT(configWidgetCurrentConfigModifiedChanged(bool)));
  connect(ui_->configWidget, SIGNAL(currentConfigUrlChanged(const QString&)),
    this, SLOT(configWidgetCurrentConfigUrlChanged(const QString&)));

  configWidgetCurrentConfigUrlChanged(QString());

  messageTypeRegistry_->update();
  packageRegistry_->update();
}

MultiplotWidget::~MultiplotWidget() {
  confirmClose();

  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

MultiplotConfig* MultiplotWidget::getConfig() const {
  return config_;
}

QDockWidget* MultiplotWidget::getDockWidget() const {
  QDockWidget* dockWidget = 0;
  QObject* currentParent = parent();

  while (currentParent) {
    dockWidget = qobject_cast<QDockWidget*>(currentParent);

    if (dockWidget)
      break;

    currentParent = currentParent->parent();
  }

  return dockWidget;
}

void MultiplotWidget::setMaxConfigHistoryLength(size_t length) {
  ui_->configWidget->setMaxConfigUrlHistoryLength(length);
}

size_t MultiplotWidget::getMaxConfigHistoryLength() const {
  return ui_->configWidget->getMaxConfigUrlHistoryLength();
}

void MultiplotWidget::setConfigHistory(const QStringList& history) {
  ui_->configWidget->setConfigUrlHistory(history);
}

QStringList MultiplotWidget::getConfigHistory() const {
  return ui_->configWidget->getConfigUrlHistory();
}

void MultiplotWidget::runPlots() {
  ui_->plotTableConfigWidget->runPlots();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MultiplotWidget::loadConfig(const QString& url) {
  ui_->configWidget->loadConfig(url);
}


void MultiplotWidget::readBag(const QString& url) {
//   ui_->bagReaderWidget->readBag(url);
}

bool MultiplotWidget::confirmClose() {
  return ui_->configWidget->confirmSave(false);
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MultiplotWidget::configWidgetCurrentConfigModifiedChanged(bool
    modified) {
  configWidgetCurrentConfigUrlChanged(ui_->configWidget->
    getCurrentConfigUrl());
}

void MultiplotWidget::configWidgetCurrentConfigUrlChanged(const QString&
    url) {
  QString windowTitle = "Multiplot";

  if (!url.isEmpty())
    windowTitle += " - ["+url+"]";
  else
    windowTitle += " - [untitled]";

  if (ui_->configWidget->isCurrentConfigModified())
    windowTitle += "*";

  setWindowTitle(windowTitle);
}

}
