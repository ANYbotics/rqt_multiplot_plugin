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

#include <ui_PlotLegendConfigWidget.h>

#include "rqt_multiplot/PlotLegendConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotLegendConfigWidget::PlotLegendConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::PlotLegendConfigWidget()),
  config_(0) {
  ui_->setupUi(this);
  
  connect(ui_->checkBoxVisible, SIGNAL(stateChanged(int)),
    this, SLOT(checkBoxVisibleStateChanged(int)));
}

PlotLegendConfigWidget::~PlotLegendConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotLegendConfigWidget::setConfig(PlotLegendConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(visibleChanged(bool)),
        this, SLOT(configVisibleChanged(bool)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(visibleChanged(bool)),
        this, SLOT(configVisibleChanged(bool)));
      
      configVisibleChanged(config->isVisible());
    }
  }
}

PlotLegendConfig* PlotLegendConfigWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotLegendConfigWidget::configVisibleChanged(bool visible) {
  ui_->checkBoxVisible->setCheckState(visible ? Qt::Checked :
    Qt::Unchecked);
}

void PlotLegendConfigWidget::checkBoxVisibleStateChanged(int state) {
  if (config_)
    config_->setVisible(state == Qt::Checked);
}

}
