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

#include "rqt_multiplot/PlotConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotConfig::PlotConfig(QObject* parent, const QString& title) :
  QObject(parent),
  title_(title) {
}

PlotConfig::~PlotConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotConfig::setTitle(const QString& title) {
  if (title != title_) {
    title_ = title;
    
    emit titleChanged(title);
    emit changed();
  }
}

const QString& PlotConfig::getTitle() const {
  return title_;
}

size_t PlotConfig::getNumCurves() const {
  return curveConfig_.count();
}

CurveConfig* PlotConfig::getCurveConfig(size_t index) const {
  if (index < curveConfig_.count())
    return curveConfig_[index];
  else
    return 0;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

CurveConfig* PlotConfig::addCurve() {
  CurveConfig* curveConfig = new CurveConfig(this);
  curveConfig->getColor()->setAutoColorIndex(curveConfig_.count());
  
  curveConfig_.append(curveConfig);
  connect(curveConfig, SIGNAL(changed()), this, SLOT(curveConfigChanged()));
  
  emit changed();
  
  return curveConfig;
}

void PlotConfig::removeCurve(CurveConfig* curveConfig) {
  int index = curveConfig_.indexOf(curveConfig);
  
  if (index >= 0)
    removeCurve(index);
}

void PlotConfig::removeCurve(size_t index) {
  if (index < curveConfig_.count()) {
    delete curveConfig_[index];
    
    curveConfig_.remove(index);
    
    for (size_t i = 0; i < curveConfig_.count(); ++i)
      curveConfig_[i]->getColor()->setAutoColorIndex(i);
    
    emit changed();
  }
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

PlotConfig& PlotConfig::operator=(const PlotConfig& src) {
  setTitle(src.title_);
  
  while (curveConfig_.count() < src.curveConfig_.count())
    addCurve();
  while (curveConfig_.count() > src.curveConfig_.count())
    removeCurve(curveConfig_.count()-1);
  
  for (size_t index = 0; index < curveConfig_.count(); ++index)
    *curveConfig_[index] = *src.curveConfig_[index];
  
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotConfig::curveConfigChanged() {
  emit changed();
}

}
