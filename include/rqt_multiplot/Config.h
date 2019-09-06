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

#ifndef RQT_MULTIPLOT_CONFIG_H
#define RQT_MULTIPLOT_CONFIG_H

#include <QDataStream>
#include <QObject>
#include <QSettings>

namespace rqt_multiplot {
  class Config :
    public QObject {
  Q_OBJECT
  public:
    Config(QObject* parent = 0);
    ~Config();

    virtual void save(QSettings& settings) const = 0;
    virtual void load(QSettings& settings) = 0;
    virtual void reset() = 0;
    
    virtual void write(QDataStream& stream) const = 0;
    virtual void read(QDataStream& stream) = 0;
    
  signals:
    void changed();
  };
  
  QDataStream& operator<<(QDataStream& stream, const Config& config);
  QDataStream& operator>>(QDataStream& stream, Config& config);
};

#endif
