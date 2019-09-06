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

#ifndef RQT_MULTIPLOT_MESSAGE_FIELD_COMPLETER_H
#define RQT_MULTIPLOT_MESSAGE_FIELD_COMPLETER_H

#include <QCompleter>

#include <rqt_multiplot/MessageFieldItemModel.h>

namespace rqt_multiplot {
  class MessageFieldCompleter :
    public QCompleter {
  Q_OBJECT
  public:
    MessageFieldCompleter(QObject* parent = 0);
    MessageFieldCompleter(MessageFieldItemModel* model, QObject* parent = 0);
    virtual ~MessageFieldCompleter();
  
    QStringList splitPath(const QString& path) const;
    QString pathFromIndex(const QModelIndex& index) const;
  };
};

#endif
