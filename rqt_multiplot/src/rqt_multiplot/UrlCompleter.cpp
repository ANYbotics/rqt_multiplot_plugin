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

#include "rqt_multiplot/UrlCompleter.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

UrlCompleter::UrlCompleter(QObject* parent) :
  QCompleter(parent),
  model_(new UrlItemModel(this)) {
  setModel(model_);
  
  connect(model_, SIGNAL(urlLoaded(const QString&)), this,
    SLOT(modelUrlLoaded(const QString&)));
}

UrlCompleter::~UrlCompleter() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

UrlItemModel* UrlCompleter::getModel() const {
  return model_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

QStringList UrlCompleter::splitPath(const QString& url) const {
  QString scheme, path;
  
  QStringList urlParts = url.split("://");
  
  if (urlParts.count() > 1) {
    scheme = urlParts[0];
    path = urlParts[1];
  }
  else
    path = url;
  
  QStringList pathParts = path.split("/");

  if (path[0] == '/')
    pathParts[0] = "/";

  QStringList parts;
  if (!scheme.isEmpty())
    parts.append(scheme+"://");
  parts.append(pathParts);
  
  return parts;
}

QString UrlCompleter::pathFromIndex(const QModelIndex& index) const {
  return model_->getUrl(index);
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void UrlCompleter::modelUrlLoaded(const QString& url) {
  QString prefix = completionPrefix();
  
  if (prefix.startsWith(url) && (prefix != (url+"/")))
    complete();
}

}
