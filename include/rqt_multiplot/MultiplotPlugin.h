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

#ifndef RQT_MULTIPLOT_MULTIPLOT_PLUGIN_H
#define RQT_MULTIPLOT_MULTIPLOT_PLUGIN_H

#include <QStringList>

#include <rqt_gui_cpp/plugin.h>

namespace rqt_multiplot {
  class MultiplotWidget;

  class MultiplotPlugin :
    public rqt_gui_cpp::Plugin {
  Q_OBJECT
  public:
    MultiplotPlugin();
    virtual ~MultiplotPlugin();

    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& pluginSettings,
      qt_gui_cpp::Settings& instanceSettings) const;
    void restoreSettings(const qt_gui_cpp::Settings& pluginSettings,
      const qt_gui_cpp::Settings& instanceSettings);

  private:
    MultiplotWidget* widget_;
    bool runAllPlotsOnStart_;

    void parseArguments(const QStringList& arguments);
  };
};

#endif
