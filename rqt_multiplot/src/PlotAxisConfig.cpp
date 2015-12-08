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

#include "rqt_multiplot/PlotAxisConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotAxisConfig::PlotAxisConfig(QObject* parent, bool titleVisible) :
  QObject(parent),
  titleVisible_(titleVisible) {
}

PlotAxisConfig::~PlotAxisConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotAxisConfig::setTitleVisible(bool visible) {
  if (visible != titleVisible_) {
    titleVisible_ = visible;
    
    emit titleVisibleChanged(visible);
    emit changed();
  }
}

bool PlotAxisConfig::isTitleVisible() const {
  return titleVisible_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotAxisConfig::save(QSettings& settings) const {
  settings.setValue("title_visible", titleVisible_);
}

void PlotAxisConfig::load(QSettings& settings) {
  setTitleVisible(settings.value("title_visible", true).toBool());
}

void PlotAxisConfig::reset() {
  setTitleVisible(true);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

PlotAxisConfig& PlotAxisConfig::operator=(const PlotAxisConfig& src) {
  setTitleVisible(src.titleVisible_);
  
  return *this;
}

}
