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

#ifndef RQT_MULTIPLOT_MATCH_FILTER_COMPLETER_MODEL_H
#define RQT_MULTIPLOT_MATCH_FILTER_COMPLETER_MODEL_H

#include <QSortFilterProxyModel>
#include <QString>

namespace rqt_multiplot {
  class MatchFilterCompleterModel :
    public QSortFilterProxyModel {
  Q_OBJECT
  public:
    MatchFilterCompleterModel(QObject* parent = 0, Qt::MatchFlags
      filterMatchFlags = Qt::MatchStartsWith, const QString&
      filterKey = QString());
    virtual ~MatchFilterCompleterModel();
  
    void setFilterMatchFlags(Qt::MatchFlags flags);
    Qt::MatchFlags getFilterMatchFlags() const;
    void setFilterKey(const QString& key);
    const QString& getFilterKey() const;
    
  protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex& sourceParent)
      const;

  private:
    Qt::MatchFlags filterMatchFlags_;
    QString filterKey_;
  };
};

#endif
