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

#include "rqt_multiplot/MessageFieldCompleter.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldCompleter::MessageFieldCompleter(QObject* parent) :
  QCompleter(parent) {
}
  
MessageFieldCompleter::MessageFieldCompleter(MessageFieldItemModel* model,
    QObject* parent) :
  QCompleter(model, parent) {
}

MessageFieldCompleter::~MessageFieldCompleter() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

QStringList MessageFieldCompleter::splitPath(const QString& path) const {
  MessageFieldItemModel* messageFieldItemModel = qobject_cast<
    MessageFieldItemModel*>(model());
    
  if (messageFieldItemModel)
    messageFieldItemModel->update(path);
  
  return path.split("/");
}

QString MessageFieldCompleter::pathFromIndex(const QModelIndex& index) const {
  QStringList fieldNames;
  QModelIndex fieldIndex = index;
  
  if (model()) {
    while (fieldIndex.isValid()) {
      fieldNames.prepend(model()->data(fieldIndex, Qt::DisplayRole).
        toString());
      fieldIndex = fieldIndex.parent();
    }
  }
      
  return fieldNames.join("/");
}

}
