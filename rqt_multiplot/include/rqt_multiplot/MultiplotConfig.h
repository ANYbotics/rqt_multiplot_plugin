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

#ifndef RQT_MULTIPLOT_MULTIPLOT_CONFIG_H
#define RQT_MULTIPLOT_MULTIPLOT_CONFIG_H

#include <rqt_multiplot/Config.h>
#include <rqt_multiplot/PlotTableConfig.h>

namespace rqt_multiplot {
  class MultiplotConfig :
    public Config {
  Q_OBJECT
  public:
    MultiplotConfig(QObject* parent);
    ~MultiplotConfig();

    PlotTableConfig* getTableConfig() const;
    
    MultiplotConfig& operator=(const MultiplotConfig& src);
    
    void save(QSettings& settings) const;
    void load(QSettings& settings);
    void reset();
    
    void write(QDataStream& stream) const;
    void read(QDataStream& stream);
    
  private:
    PlotTableConfig* tableConfig_;
    
  private slots:
    void tableConfigChanged();
  };
};

#endif
