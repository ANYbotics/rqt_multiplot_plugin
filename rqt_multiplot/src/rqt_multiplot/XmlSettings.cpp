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

#include <QMap>
#include <QList>
#include <QSharedPointer>
#include <QStringList>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

#include "rqt_multiplot/XmlSettings.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Static Initializations                                                    */
/*****************************************************************************/

const QSettings::Format XmlSettings::format = QSettings::registerFormat("xml",
  XmlSettings::read, XmlSettings::write);
   
/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool XmlSettings::read(QIODevice& device, QSettings::SettingsMap& map) {
  QXmlStreamReader xmlReader(&device);

  QStringList groups;
  
  while (!xmlReader.atEnd()) {
    xmlReader.readNext();
    
    if (xmlReader.isStartElement())
      groups.append(xmlReader.name().toString());      
    else if (xmlReader.isCharacters() && !xmlReader.isWhitespace())
      map[groups.join("/")] = xmlReader.text().toString();
    else if (xmlReader.isEndElement())
      groups.removeLast();
  }

  return !xmlReader.hasError();
}

bool XmlSettings::write(QIODevice& device, const QSettings::SettingsMap& map) {
  struct NestedMap;
  struct NestedMap : QMap<QString, QSharedPointer<NestedMap> > {};

  QSharedPointer<NestedMap> nestedMap(new NestedMap());
  
  for (QSettings::SettingsMap::const_iterator it= map.begin();
      it != map.end(); ++it) {
    QSharedPointer<NestedMap> currentMap = nestedMap;
  
    QStringList groups = it.key().split("/");
  
    for (QStringList::const_iterator jt = groups.begin();
        jt != groups.end(); ++jt) {
      NestedMap::iterator kt = currentMap->find(*jt);
    
      if (kt == currentMap->end()) {
        kt = currentMap->insert(*jt, QSharedPointer<NestedMap>(
          new NestedMap()));
        currentMap = kt.value();
      }
      else
        currentMap = kt.value();
    }
  }
  
  QXmlStreamWriter xmlWriter(&device);
  
  xmlWriter.setAutoFormatting(true);
  xmlWriter.writeStartDocument();
 
  QStringList groups;
  QList<QSharedPointer<NestedMap> > nestedMaps;
  QList<NestedMap::iterator> nestedMapIterators;
  
  nestedMaps.append(nestedMap);
  nestedMapIterators.append(nestedMap->begin());

  while (!nestedMaps.isEmpty()) {
    QSharedPointer<NestedMap> currentMap = nestedMaps.last();
    NestedMap::iterator it = nestedMapIterators.last();
    
    if (it != currentMap->end()) {
      xmlWriter.writeStartElement(it.key());
      
      groups.append(it.key());
      nestedMaps.append(it.value());
      nestedMapIterators.append(it.value()->begin());
    }
    else {
      if (currentMap->isEmpty())
        xmlWriter.writeCharacters(map[groups.join("/")].toString());

      xmlWriter.writeEndElement();
      
      if (!groups.isEmpty())
        groups.removeLast();
      nestedMaps.removeLast();
      nestedMapIterators.removeLast();
      
      if (!nestedMaps.isEmpty())
        ++nestedMapIterators.last();
    }
  }
  
  xmlWriter.writeEndDocument();

  return true;
}

}
