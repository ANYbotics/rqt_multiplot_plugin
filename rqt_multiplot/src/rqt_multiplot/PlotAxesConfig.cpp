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

#include "rqt_multiplot/PlotAxesConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotAxesConfig::PlotAxesConfig(QObject* parent) :
  Config(parent) {
  axisConfig_[X] = new PlotAxisConfig(this);
  axisConfig_[Y] = new PlotAxisConfig(this);
    
  connect(axisConfig_[X], SIGNAL(changed()), this, SLOT(axisConfigChanged()));
  connect(axisConfig_[Y], SIGNAL(changed()), this, SLOT(axisConfigChanged()));
}

PlotAxesConfig::~PlotAxesConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

PlotAxisConfig* PlotAxesConfig::getAxisConfig(Axis axis) const {
  QMap<Axis, PlotAxisConfig*>::const_iterator it = axisConfig_.find(axis);
  
  if (it != axisConfig_.end())
    return it.value();
  else
    return 0;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotAxesConfig::save(QSettings& settings) const {
  settings.beginGroup("axes");
  settings.beginGroup("x_axis");
  axisConfig_[X]->save(settings);
  settings.endGroup();
  settings.beginGroup("y_axis");
  axisConfig_[Y]->save(settings);
  settings.endGroup();
  settings.endGroup();
}

void PlotAxesConfig::load(QSettings& settings) {
  settings.beginGroup("axes");
  settings.beginGroup("x_axis");
  axisConfig_[X]->load(settings);
  settings.endGroup();
  settings.beginGroup("y_axis");
  axisConfig_[Y]->load(settings);
  settings.endGroup();
  settings.endGroup();
}

void PlotAxesConfig::reset() {
  axisConfig_[X]->reset();
  axisConfig_[Y]->reset();
}

void PlotAxesConfig::write(QDataStream& stream) const {
  axisConfig_[X]->write(stream);
  axisConfig_[Y]->write(stream);
}

void PlotAxesConfig::read(QDataStream& stream) {
  axisConfig_[X]->read(stream);
  axisConfig_[Y]->read(stream);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

PlotAxesConfig& PlotAxesConfig::operator=(const PlotAxesConfig& src) {
  *axisConfig_[X] = *src.axisConfig_[X];
  *axisConfig_[Y] = *src.axisConfig_[Y];
  
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotAxesConfig::axisConfigChanged() {
  emit changed();
}

}
