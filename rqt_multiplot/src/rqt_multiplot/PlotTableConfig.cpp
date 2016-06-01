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
    backgroundColor, const QColor& foregroundColor, size_t numRows,
    size_t numColumns, bool linkScale, bool linkCursor, bool trackPoints) :
  Config(parent),
  backgroundColor_(backgroundColor),
  foregroundColor_(foregroundColor),
  linkScale_(linkScale),
  linkCursor_(linkCursor),
  trackPoints_(trackPoints) {
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

void PlotTableConfig::setForegroundColor(const QColor& color) {
  if (color != foregroundColor_) {
    foregroundColor_ = color;
    
    emit foregroundColorChanged(color);
    emit changed();
  }
}

const QColor& PlotTableConfig::getForegroundColor() const {
  return foregroundColor_;
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

void PlotTableConfig::setLinkScale(bool link) {
  if (link != linkScale_) {
    linkScale_ = link;
    
    emit linkScaleChanged(link);
    emit changed();
  }
}

bool PlotTableConfig::isScaleLinked() const {
  return linkScale_;
}

void PlotTableConfig::setLinkCursor(bool link) {
  if (link != linkCursor_) {
    linkCursor_ = link;
    
    emit linkCursorChanged(link);
    emit changed();
  }
}

bool PlotTableConfig::isCursorLinked() const {
  return linkCursor_;
}

void PlotTableConfig::setTrackPoints(bool track) {
  if (track != trackPoints_) {
    trackPoints_ = track;
    
    emit trackPointsChanged(track);
    emit changed();
  }
}

bool PlotTableConfig::arePointsTracked() const {
  return trackPoints_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotTableConfig::save(QSettings& settings) const {
  settings.setValue("background_color", QVariant::fromValue<QColor>(
    backgroundColor_));
  settings.setValue("foreground_color", QVariant::fromValue<QColor>(
    foregroundColor_));
  
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
  
  settings.setValue("link_scale", linkScale_);
  settings.setValue("link_cursor", linkCursor_);
  settings.setValue("track_points", trackPoints_);
}

void PlotTableConfig::load(QSettings& settings) {
  setBackgroundColor(settings.value("background_color", (QColor)Qt::white).
    value<QColor>());
  setForegroundColor(settings.value("foreground_color", (QColor)Qt::black).
    value<QColor>());
  
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
  
  setLinkScale(settings.value("link_scale", false).toBool());
  setLinkCursor(settings.value("link_cursor", false).toBool());
  setTrackPoints(settings.value("track_points", false).toBool());
}

void PlotTableConfig::reset() {
  setBackgroundColor(Qt::white);
  setForegroundColor(Qt::black);

  setNumPlots(1, 1);
  plotConfig_[0][0]->reset();
  
  setLinkScale(false);
  setLinkCursor(false);
  setTrackPoints(false);
}

void PlotTableConfig::write(QDataStream& stream) const {
  stream << backgroundColor_;
  stream << foregroundColor_;
  
  stream << (quint64)getNumRows() << (quint64)getNumColumns();
  
  for (size_t row = 0; row < plotConfig_.count(); ++row)
    for (size_t column = 0; column < plotConfig_[row].count(); ++column)
      plotConfig_[row][column]->write(stream);
  
  stream << linkScale_;
  stream << linkCursor_;
  stream << trackPoints_;
}

void PlotTableConfig::read(QDataStream& stream) {
  QColor backgroundColor, foregroundColor;
  bool linkScale, linkCursor, trackPoints;
  quint64 numRows, numColumns;
  
  stream >> backgroundColor;
  setBackgroundColor(backgroundColor);
  stream >> foregroundColor;
  setForegroundColor(foregroundColor);
  
  stream >> numRows >> numColumns;
  setNumPlots(numRows, numColumns);
  for (size_t row = 0; row < plotConfig_.count(); ++row)
    for (size_t column = 0; column < plotConfig_[row].count(); ++column)
      plotConfig_[row][column]->read(stream);
  
  stream >> linkScale;
  setLinkScale(linkScale);
  stream >> linkCursor;
  setLinkCursor(linkCursor);
  stream >> trackPoints;
  setTrackPoints(trackPoints);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

PlotTableConfig& PlotTableConfig::operator=(const PlotTableConfig& src) {
  setBackgroundColor(src.backgroundColor_);
  setForegroundColor(src.foregroundColor_);
  
  setNumPlots(src.getNumRows(), src.getNumColumns());
  
  for (size_t row = 0; row < getNumRows(); ++row)
    for (size_t column = 0; column < getNumColumns(); ++column)
      *plotConfig_[row][column] = *src.plotConfig_[row][column];

  setLinkScale(src.linkScale_);
  setLinkCursor(src.linkCursor_);
  setTrackPoints(src.trackPoints_);
    
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotTableConfig::plotConfigChanged() {
  emit changed();
}

}
