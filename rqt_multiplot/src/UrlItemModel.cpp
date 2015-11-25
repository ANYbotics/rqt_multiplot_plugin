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

#include "rqt_multiplot/UrlItemModel.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

UrlItemModel::UrlItemModel(QObject* parent) {
}

UrlItemModel::~UrlItemModel() {
  for (QList<UrlItem*>::iterator it = schemeItems_.begin();
      it != schemeItems_.end(); ++it)
    delete *it;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

QString UrlItemModel::getUrl(const QModelIndex& index) const {
  if (index.isValid()) {
    UrlItem* item = static_cast<UrlItem*>(index.internalPointer());
    UrlScheme* itemScheme = item->getScheme();

    if (item->getType() == UrlItem::Scheme)
      return itemScheme->getPrefix()+"://";
    else if (item->getType() == UrlItem::Host)
      return itemScheme->getPrefix()+"://"+itemScheme->getHost(
        item->getIndex());
    else if (item->getType() == UrlItem::Path) {
      QModelIndex hostIndex = item->getIndex(UrlItem::Host);
      
      return itemScheme->getPrefix()+"://"+itemScheme->getHost(hostIndex)+
        "/"+itemScheme->getPath(hostIndex, item->getIndex());
    }
  }
  
  return QString();
}

QString UrlItemModel::getFilePath(const QModelIndex& index) const {
  if (index.isValid()) {
    UrlItem* item = static_cast<UrlItem*>(index.internalPointer());
    UrlScheme* itemScheme = item->getScheme();

    if (item->getType() == UrlItem::Host)
      return itemScheme->getFilePath(item->getIndex(), QModelIndex());
    else if (item->getType() == UrlItem::Path) {
      QModelIndex hostIndex = item->getIndex(UrlItem::Host);
      
      return itemScheme->getFilePath(hostIndex, item->getIndex());
    }
  }
  
  return QString();
}

QString UrlItemModel::getFilePath(const QString& url) const {
  QStringList urlParts = url.split("://");
  
  if (urlParts.count() > 1) {
    QString prefix = urlParts[0];
    
    for (QList<UrlScheme*>::const_iterator it = schemes_.begin();
        it != schemes_.end(); ++it) {
      UrlScheme* scheme = *it;
      
      if (scheme->getPrefix() == prefix) {
        QStringList hostPathParts = urlParts[1].split("/");
        QString host, path;        
        
        if (hostPathParts.count() > 1) {
          host = hostPathParts[0];
          hostPathParts.removeFirst();
          path = hostPathParts.join("/");
        }
        else
          host = urlParts[1];
        
        return scheme->getFilePath(host, path);
      }
    }
  }
  
  return QString();
}

UrlScheme* UrlItemModel::getScheme(const QModelIndex& index) const {
  if (index.isValid()) {
    UrlItem* item = static_cast<UrlItem*>(index.internalPointer());
    
    if (item)
      return item->getScheme();
  }
  
  return 0;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void UrlItemModel::addScheme(UrlScheme* scheme) {
  schemes_.append(scheme);
  schemeItems_.append(new UrlItem(scheme));
  
  connect(scheme, SIGNAL(resetStarted()), this, SLOT(schemeResetStarted()));
  connect(scheme, SIGNAL(resetFinished()), this, SLOT(schemeResetFinished()));
  connect(scheme, SIGNAL(pathLoaded(const QString&, const QString&)),
    this, SLOT(schemePathLoaded(const QString&, const QString&)));
}

int UrlItemModel::rowCount(const QModelIndex& parent) const {
  if (parent.column() <= 0) {
    if (parent.isValid()) {
      UrlItem* parentItem = static_cast<UrlItem*>(parent.internalPointer());
      UrlScheme* parentScheme = parentItem->getScheme();
      
      if (parentItem->getType() == UrlItem::Scheme) {
        size_t numHosts = parentScheme->getNumHosts();
        
        if (!numHosts)
          return parentScheme->getNumPaths(QModelIndex());
        else
          return numHosts;
      }
      else if (parentItem->getType() == UrlItem::Host)
        return parentScheme->getNumPaths(parentItem->getIndex());
      else if (parentItem->getType() == UrlItem::Path)
        return parentScheme->getNumPaths(parentItem->getIndex(
          UrlItem::Host), parentItem->getIndex());
    }
    else
      return schemes_.count();
  }

  return 0;
}

int UrlItemModel::columnCount(const QModelIndex& parent) const {
  return 1;
}

QVariant UrlItemModel::data(const QModelIndex& index, int role) const {
  if (index.isValid()) {
    UrlItem* item = static_cast<UrlItem*>(index.internalPointer());
    UrlScheme* itemScheme = item->getScheme();

    if (item->getType() == UrlItem::Scheme) {
      if ((role == Qt::DisplayRole) || (role == Qt::EditRole))
        return itemScheme->getPrefix()+"://";
    }
    else if (item->getType() == UrlItem::Host)
      return itemScheme->getHostData(item->getIndex(), role);
    else if (item->getType() == UrlItem::Path)
      return itemScheme->getPathData(item->getIndex(), role);
  }

  return QVariant();
}

QModelIndex UrlItemModel::index(int row, int column, const QModelIndex& parent)
    const {
  if (hasIndex(row, column, parent)) {
    if (parent.isValid()) {
      UrlItem* parentItem = static_cast<UrlItem*>(parent.internalPointer());
      UrlScheme* parentScheme = parentItem->getScheme();
      UrlItem* childItem = 0;

      if (parentItem->getType() == UrlItem::Scheme) {
        size_t numHosts = parentScheme->getNumHosts();
        
        if (!numHosts)
          childItem = parentItem->addChild(row, UrlItem::Path,
            parentScheme->getPathIndex(QModelIndex(), row));
        else
          childItem = parentItem->addChild(row, UrlItem::Host,
            parentScheme->getHostIndex(row));
      }
      else if (parentItem->getType() == UrlItem::Host)
        childItem = parentItem->addChild(row, UrlItem::Path,
          parentScheme->getPathIndex(parentItem->getIndex(UrlItem::Host),
          row));
      else if (parentItem->getType() == UrlItem::Path)
        childItem = parentItem->addChild(row, UrlItem::Path,
          parentScheme->getPathIndex(parentItem->getIndex(UrlItem::Host),
          row, parentItem->getIndex()));
        
      return createIndex(row, column, childItem);
    }
    else
      return createIndex(row, column, schemeItems_[row]);
  }

  return QModelIndex();
}

QModelIndex UrlItemModel::parent(const QModelIndex& index) const {
  if (index.isValid()) {
    UrlItem* childItem = static_cast<UrlItem*>(index.internalPointer());

    if (childItem) {
      UrlItem* parentItem = childItem->getParent();

      if (parentItem)
        return createIndex(parentItem->getRow(), 0, parentItem);
    }
  }
  
  return QModelIndex();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void UrlItemModel::schemeResetStarted() {
  beginResetModel();
  
  UrlScheme* scheme = static_cast<UrlScheme*>(sender());
  int i = schemes_.indexOf(scheme);
  
  if (i >= 0) {
    delete schemeItems_[i];
    schemeItems_[i] = new UrlItem(scheme);
  }
}

void UrlItemModel::schemeResetFinished() {
  endResetModel();
}

void UrlItemModel::schemePathLoaded(const QString& host, const QString&
    path) {
  UrlScheme* scheme = static_cast<UrlScheme*>(sender());

  QString url = scheme->getPrefix()+"://"+host;
  
  if (!path.isEmpty())
    url += "/"+path;
  
  emit urlLoaded(url);
}

}
