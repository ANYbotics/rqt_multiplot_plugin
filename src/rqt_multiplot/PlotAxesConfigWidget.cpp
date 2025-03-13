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

#include <ui_PlotAxesConfigWidget.h>

#include "rqt_multiplot/PlotAxesConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotAxesConfigWidget::PlotAxesConfigWidget(QWidget* parent) : QWidget(parent), ui_(new Ui::PlotAxesConfigWidget()), config_(nullptr) {
  ui_->setupUi(this);
}

PlotAxesConfigWidget::~PlotAxesConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotAxesConfigWidget::setConfig(PlotAxesConfig* config) {
  if (config != config_) {
    config_ = config;

    if (config != nullptr) {
      ui_->plotAxisConfigWidgetX->setConfig(config_->getAxisConfig(PlotAxesConfig::X));
      ui_->plotAxisConfigWidgetY->setConfig(config_->getAxisConfig(PlotAxesConfig::Y));
    } else {
      ui_->plotAxisConfigWidgetX->setConfig(nullptr);
      ui_->plotAxisConfigWidgetY->setConfig(nullptr);
    }
  }
}

PlotAxesConfig* PlotAxesConfigWidget::getConfig() const {
  return config_;
}

}  // namespace rqt_multiplot
