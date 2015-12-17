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

PlotAxisConfig::PlotAxisConfig(QObject* parent, TitleType titleType, const
    QString& customTitle, bool titleVisible) :
  Config(parent),
  titleType_(titleType),
  customTitle_(customTitle),
  titleVisible_(titleVisible) {
}

PlotAxisConfig::~PlotAxisConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotAxisConfig::setTitleType(TitleType type) {
  if (type != titleType_) {
    titleType_ = type;
    
    emit titleTypeChanged(type);
    emit changed();
  }
}

PlotAxisConfig::TitleType PlotAxisConfig::getTitleType() const {
  return titleType_;
}

void PlotAxisConfig::setCustomTitle(const QString& title) {
  if (title != customTitle_) {
    customTitle_ = title;
    
    emit customTitleChanged(title);
    emit changed();
  }
}

const QString& PlotAxisConfig::getCustomTitle() const {
  return customTitle_;
}

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
  settings.setValue("title_type", titleType_);
  settings.setValue("custom_title", customTitle_);
  settings.setValue("title_visible", titleVisible_);
}

void PlotAxisConfig::load(QSettings& settings) {
  setTitleType(static_cast<TitleType>(settings.value("title_type",
    AutoTitle).toInt()));
  setCustomTitle(settings.value("custom_title", "Untitled Axis").toString());
  setTitleVisible(settings.value("title_visible", true).toBool());
}

void PlotAxisConfig::reset() {
  setTitleType(AutoTitle);
  setCustomTitle("Untitled Axis");
  setTitleVisible(true);
}

void PlotAxisConfig::write(QDataStream& stream) const {
  stream << (int)titleType_;
  stream << customTitle_;
  stream << titleVisible_;
}

void PlotAxisConfig::read(QDataStream& stream) {
  int titleType;
  QString customTitle;
  bool titleVisible;
  
  stream >> titleType;
  setTitleType(static_cast<TitleType>(titleType));
  stream >> customTitle;
  setCustomTitle(customTitle);
  stream >> titleVisible;
  setTitleVisible(titleVisible);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

PlotAxisConfig& PlotAxisConfig::operator=(const PlotAxisConfig& src) {
  setTitleType(src.titleType_);
  setCustomTitle(src.customTitle_);
  setTitleVisible(src.titleVisible_);
  
  return *this;
}

}
