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

#include <QStringList>

#include "rqt_multiplot/MatchFilterCompleter.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MatchFilterCompleter::MatchFilterCompleter(QObject* parent, Qt::MatchFlags
    filterMatchFlags) :
  QCompleter(parent),
  proxyModel_(new MatchFilterCompleterModel(this, filterMatchFlags)) {
  setCompletionMode(QCompleter::UnfilteredPopupCompletion);
}

MatchFilterCompleter::~MatchFilterCompleter() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MatchFilterCompleter::setFilterMatchFlags(Qt::MatchFlags flags) {
  proxyModel_->setFilterMatchFlags(flags);
}

Qt::MatchFlags MatchFilterCompleter::getFilterMatchFlags() const {
  return proxyModel_->getFilterMatchFlags();
}

void MatchFilterCompleter::setFilterCaseSensitivity(Qt::CaseSensitivity
    caseSensitivity) {
  proxyModel_->setFilterCaseSensitivity(caseSensitivity);
}

Qt::CaseSensitivity MatchFilterCompleter::getFilterCaseSensitivity() const {
  return proxyModel_->filterCaseSensitivity();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

QStringList MatchFilterCompleter::splitPath(const QString& path) const {
  QAbstractItemModel* sourceModel = model();
  
  if (sourceModel && (sourceModel != proxyModel_)) {
    sourceModel->setParent(proxyModel_);
    proxyModel_->setSourceModel(sourceModel);
    
    const_cast<MatchFilterCompleter*>(this)->setModel(proxyModel_);
  }
  
  proxyModel_->setFilterKey(path);
  
  return QStringList();
}

}
