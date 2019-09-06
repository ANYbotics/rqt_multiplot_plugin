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

#ifndef RQT_MULTIPLOT_URL_COMPLETER_H
#define RQT_MULTIPLOT_URL_COMPLETER_H

#include <QCompleter>

#include <rqt_multiplot/UrlItemModel.h>

namespace rqt_multiplot {
  class UrlCompleter :
    public QCompleter {
  Q_OBJECT
  public:
    UrlCompleter(QObject* parent = 0);
    virtual ~UrlCompleter();
    
    UrlItemModel* getModel() const;
    
    QStringList splitPath(const QString& url) const;
    QString pathFromIndex(const QModelIndex& index) const;
    
  private:
    UrlItemModel* model_;
    
  private slots:
    void modelUrlLoaded(const QString& url);
  };
};

#endif
