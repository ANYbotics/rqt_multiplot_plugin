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

PlotConfig::PlotConfig(QObject* parent, const QString& title, double
    plotRate) :
  QObject(parent),
  title_(title),
  axesConfig_(new PlotAxesConfig(this)),
  legendConfig_(new PlotLegendConfig(this)),
  plotRate_(plotRate) {
  connect(axesConfig_, SIGNAL(changed()), this,
    SLOT(axesConfigChanged()));
  connect(legendConfig_, SIGNAL(changed()), this,
    SLOT(legendConfigChanged()));
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

PlotAxesConfig* PlotConfig::getAxesConfig() const {
  return axesConfig_;
}

PlotLegendConfig* PlotConfig::getLegendConfig() const {
  return legendConfig_;
}

void PlotConfig::setPlotRate(double rate) {
  if (rate != plotRate_) {
    plotRate_ = rate;
    
    emit plotRateChanged(rate);
    emit changed();
  }
}

double PlotConfig::getPlotRate() const {
  return plotRate_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

CurveConfig* PlotConfig::addCurve() {
  CurveConfig* curveConfig = new CurveConfig(this);
  curveConfig->getColor()->setAutoColorIndex(curveConfig_.count());
  
  curveConfig_.append(curveConfig);
  connect(curveConfig, SIGNAL(changed()), this, SLOT(curveConfigChanged()));
  
  emit curveAdded(curveConfig_.count()-1);
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
    
    emit curveRemoved(index);
    emit changed();
  }
}

void PlotConfig::clearCurves() {
  if (!curveConfig_.isEmpty()) {
    for (size_t i = 0; i < curveConfig_.count(); ++i)
      delete curveConfig_[i];
    
    curveConfig_.clear();
    
    emit curvesCleared();
    emit changed();
  }
}

void PlotConfig::save(QSettings& settings) const {
  settings.setValue("title", title_);
  
  settings.beginGroup("curves");
  
  for (size_t index = 0; index < curveConfig_.count(); ++index) {
    settings.beginGroup("curve_"+QString::number(index));
    curveConfig_[index]->save(settings);
    settings.endGroup();
  }
  
  settings.endGroup();

  settings.beginGroup("axes");
  axesConfig_->save(settings);
  settings.endGroup();
  
  settings.beginGroup("legend");
  legendConfig_->save(settings);
  settings.endGroup();
  
  settings.setValue("plot_rate", plotRate_);  
}

void PlotConfig::load(QSettings& settings) {
  setTitle(settings.value("title", "Untitled Curve").toString());
  
  settings.beginGroup("curves");
  
  QStringList groups = settings.childGroups();
  size_t index = 0;
  
  for (QStringList::iterator it = groups.begin(); it != groups.end(); ++it) {
    CurveConfig* curveConfig = 0;
    
    if (index < curveConfig_.count())
      curveConfig = curveConfig_[index];
    else
      curveConfig = addCurve();
    
    settings.beginGroup(*it);
    curveConfig->load(settings);
    settings.endGroup();
    
    ++index;
  }

  settings.endGroup();
  
  while (index < curveConfig_.count())
    removeCurve(index);
  
  settings.beginGroup("axes");
  axesConfig_->load(settings);
  settings.endGroup();
  
  settings.beginGroup("legend");
  legendConfig_->load(settings);
  settings.endGroup();
  
  setPlotRate(settings.value("plot_rate", 30.0).toDouble());
}

void PlotConfig::reset() {
  setTitle("Untitled Plot");
  clearCurves();
  axesConfig_->reset();
  legendConfig_->reset();
  setPlotRate(30.0);
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
  
  *axesConfig_ = *src.axesConfig_;
  *legendConfig_ = *src.legendConfig_;
  
  setPlotRate(src.plotRate_);
  
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotConfig::curveConfigChanged() {
  for (size_t index = 0; index < curveConfig_.count(); ++index) {
    if (curveConfig_[index] == sender()) {
      emit curveConfigChanged(index);
      
      break;
    }
  }
  
  emit changed();
}

void PlotConfig::axesConfigChanged() {
  emit changed();
}

void PlotConfig::legendConfigChanged() {
  emit changed();
}

}
