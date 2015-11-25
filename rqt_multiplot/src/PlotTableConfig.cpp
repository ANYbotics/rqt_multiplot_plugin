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

#include "rqt_multiplot/PlotTableConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotTableConfig::PlotTableConfig(QObject* parent, const QColor&
    backgroundColor, size_t numRows, size_t numColumns) :
  QObject(parent),
  backgroundColor_(backgroundColor) {
  if (numRows && numColumns) {
    plotConfig_.resize(numRows);
    
    for (size_t row = 0; row < numRows; ++row) {
      plotConfig_[row].resize(numColumns);
      
      for (size_t column = 0; column < numColumns; ++column) {
        plotConfig_[row][column] = new PlotConfig(this);
        
        connect(plotConfig_[row][column], SIGNAL(changed()), this,
          SLOT(plotConfigChanged()));
      }
    }
  }
}

PlotTableConfig::~PlotTableConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotTableConfig::setBackgroundColor(const QColor& color) {
  if (color != backgroundColor_) {
    backgroundColor_ = color;
    
    emit backgroundColorChanged(color);
    emit changed();
  }
}

const QColor& PlotTableConfig::getBackgroundColor() const {
  return backgroundColor_;
}

void PlotTableConfig::setNumPlots(size_t numRows, size_t numColumns) {
  if ((numRows != getNumRows()) || (numColumns != getNumColumns())) {
    size_t oldNumRows = getNumRows();
    size_t oldNumColumns = getNumColumns();
    
    if (!numRows || !numColumns) {
      numRows = 0;
      numColumns = 0;
    }
    
    QVector<QVector<PlotConfig*> > plotConfig(numRows);
    
    for (size_t row = 0; row < numRows; ++row) {
      plotConfig[row].resize(numColumns);
      
      for (size_t column = 0; column < numColumns; ++column) {
        if ((row < oldNumRows) && (column < oldNumColumns))
          plotConfig[row][column] = plotConfig_[row][column];
        else {
          plotConfig[row][column] = new PlotConfig(this);
          
          connect(plotConfig[row][column], SIGNAL(changed()), this,
            SLOT(plotConfigChanged()));
        }
      }
    }
    
    for (size_t row = 0; row < oldNumRows; ++row)
      for (size_t column = 0; column < oldNumColumns; ++column)
        if ((row >= numRows) || (column >= numColumns))
          delete plotConfig_[row][column];
    
    plotConfig_ = plotConfig;
    
    emit numPlotsChanged(numRows, numColumns);
    emit changed();
  }
}

void PlotTableConfig::setNumRows(size_t numRows) {
  setNumPlots(numRows, getNumColumns());
}

size_t PlotTableConfig::getNumRows() const {
  return plotConfig_.count();
}

void PlotTableConfig::setNumColumns(size_t numColumns) {
  setNumPlots(getNumRows(), numColumns);
}

size_t PlotTableConfig::getNumColumns() const {
  if (!plotConfig_.isEmpty())
    return plotConfig_[0].count();
  else
    return 0;
}

PlotConfig* PlotTableConfig::getPlotConfig(size_t row, size_t column) const {
  if ((row < getNumRows()) && (column < getNumColumns()))
    return plotConfig_[row][column];
  else
    return 0;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotTableConfig::save(QSettings& settings) const {
  settings.setValue("background_color", QVariant::fromValue<QColor>(
    backgroundColor_));
  
  settings.beginGroup("plots");
  
  for (size_t row = 0; row < plotConfig_.count(); ++row) {
    settings.beginGroup("row_"+QString::number(row));
    
    for (size_t column = 0; column < plotConfig_[row].count(); ++column) {
      settings.beginGroup("column_"+QString::number(column));
      plotConfig_[row][column]->save(settings);
      settings.endGroup();
    }
    
    settings.endGroup();
  }
  
  settings.endGroup();
}

void PlotTableConfig::load(QSettings& settings) {
  setBackgroundColor(settings.value("background_color").value<QColor>());
  
  settings.beginGroup("plots");
  
  QStringList rowGroups = settings.childGroups();
  size_t row = 0;
  size_t numColumns = 0;
  
  for (QStringList::iterator it = rowGroups.begin();
      it != rowGroups.end(); ++it) {
    if (row >= plotConfig_.count())
      setNumRows(row+1);
    
    settings.beginGroup(*it);
    
    QStringList columnGroups = settings.childGroups();
    size_t column = 0;
    
    for (QStringList::iterator jt = columnGroups.begin();
        jt != columnGroups.end(); ++jt) {    
      if (column >= plotConfig_[row].count())
        setNumColumns(column+1);
      
      settings.beginGroup(*jt);
      plotConfig_[row][column]->load(settings);
      settings.endGroup();
      
      ++column;
    }
    
    settings.endGroup();
    
    numColumns = std::max(numColumns, column);
    ++row;
  }
    
  settings.endGroup();
  
  setNumPlots(row, numColumns);
}

void PlotTableConfig::reset() {
  setBackgroundColor(Qt::white);
  setNumPlots(1, 1);

  plotConfig_[0][0]->reset();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

PlotTableConfig& PlotTableConfig::operator=(const PlotTableConfig& src) {
  setBackgroundColor(src.backgroundColor_);
  setNumPlots(src.getNumRows(), src.getNumColumns());
  
  for (size_t row = 0; row < getNumRows(); ++row)
    for (size_t column = 0; column < getNumColumns(); ++column)
      *plotConfig_[row][column] = *src.plotConfig_[row][column];
  
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotTableConfig::plotConfigChanged() {
  emit changed();
}

}
