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

#include <ui_PlotAxisConfigWidget.h>

#include "rqt_multiplot/PlotAxisConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotAxisConfigWidget::PlotAxisConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::PlotAxisConfigWidget()),
  config_(0) {
  ui_->setupUi(this);
  
  connect(ui_->lineEditTitle, SIGNAL(editingFinished()),
    this, SLOT(lineEditTitleEditingFinished()));
  connect(ui_->checkBoxTitleAuto, SIGNAL(stateChanged(int)),
    this, SLOT(checkBoxTitleAutoStateChanged(int)));
  connect(ui_->checkBoxTitleVisible, SIGNAL(stateChanged(int)),
    this, SLOT(checkBoxTitleVisibleStateChanged(int)));
}

PlotAxisConfigWidget::~PlotAxisConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotAxisConfigWidget::setConfig(PlotAxisConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(titleTypeChanged(int)),
        this, SLOT(configTitleTypeChanged(int)));
      disconnect(config_, SIGNAL(customTitleChanged(const QString&)),
        this, SLOT(configCustomTitleChanged(const QString&)));
      disconnect(config_, SIGNAL(titleVisibleChanged(bool)),
        this, SLOT(configTitleVisibleChanged(bool)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(titleTypeChanged(int)),
        this, SLOT(configTitleTypeChanged(int)));
      connect(config, SIGNAL(customTitleChanged(const QString&)),
        this, SLOT(configCustomTitleChanged(const QString&)));
      connect(config, SIGNAL(titleVisibleChanged(bool)),
        this, SLOT(configTitleVisibleChanged(bool)));
      
      configTitleTypeChanged(config->getTitleType());
      configCustomTitleChanged(config->getCustomTitle());
      configTitleVisibleChanged(config->isTitleVisible());
    }
  }
}

PlotAxisConfig* PlotAxisConfigWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotAxisConfigWidget::configTitleTypeChanged(int type) {
  ui_->checkBoxTitleAuto->setCheckState((type == PlotAxisConfig::AutoTitle) ?
    Qt::Checked : Qt::Unchecked);
}

void PlotAxisConfigWidget::configCustomTitleChanged(const QString& title) {
  ui_->lineEditTitle->setText(title);
}

void PlotAxisConfigWidget::configTitleVisibleChanged(bool visible) {
  ui_->checkBoxTitleVisible->setCheckState(visible ? Qt::Checked :
    Qt::Unchecked);
}

void PlotAxisConfigWidget::checkBoxTitleAutoStateChanged(int state) {
  ui_->lineEditTitle->setEnabled(state != Qt::Checked);
  
  if (config_)
    config_->setTitleType((state == Qt::Checked) ? PlotAxisConfig::AutoTitle :
       PlotAxisConfig::CustomTitle);
}

void PlotAxisConfigWidget::lineEditTitleEditingFinished() {
  if (config_)
    config_->setCustomTitle(ui_->lineEditTitle->text());
}

void PlotAxisConfigWidget::checkBoxTitleVisibleStateChanged(int state) {
  if (config_)
    config_->setTitleVisible(state == Qt::Checked);
}

}
