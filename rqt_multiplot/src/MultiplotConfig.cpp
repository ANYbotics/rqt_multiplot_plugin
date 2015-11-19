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

#include "rqt_multiplot/MultiplotConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MultiplotConfig::MultiplotConfig(QObject* parent) :
  QObject(parent),
  tableConfig_(new PlotTableConfig(this)) {
  connect(tableConfig_, SIGNAL(changed()), this, SLOT(tableConfigChanged()));
}

MultiplotConfig::~MultiplotConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

PlotTableConfig* MultiplotConfig::getTableConfig() const {
  return tableConfig_;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

MultiplotConfig& MultiplotConfig::operator=(const MultiplotConfig& src) {
  *tableConfig_ = *src.tableConfig_;
  
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MultiplotConfig::tableConfigChanged() {
  emit changed();
}

}
