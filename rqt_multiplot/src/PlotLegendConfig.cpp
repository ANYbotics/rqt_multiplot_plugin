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

#include "rqt_multiplot/PlotLegendConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotLegendConfig::PlotLegendConfig(QObject* parent, bool visible) :
  Config(parent),
  visible_(visible) {
}

PlotLegendConfig::~PlotLegendConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotLegendConfig::setVisible(bool visible) {
  if (visible != visible_) {
    visible_ = visible;
    
    emit visibleChanged(visible);
    emit changed();
  }
}

bool PlotLegendConfig::isVisible() const {
  return visible_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotLegendConfig::save(QSettings& settings) const {
  settings.setValue("visible", visible_);
}

void PlotLegendConfig::load(QSettings& settings) {
  setVisible(settings.value("visible", true).toBool());
}

void PlotLegendConfig::reset() {
  setVisible(true);
}

void PlotLegendConfig::write(QDataStream& stream) const {
  stream << visible_;
}

void PlotLegendConfig::read(QDataStream& stream) {
  bool visible;
  
  stream >> visible;
  setVisible(visible);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

PlotLegendConfig& PlotLegendConfig::operator=(const PlotLegendConfig& src) {
  setVisible(src.visible_);
  
  return *this;
}

}
