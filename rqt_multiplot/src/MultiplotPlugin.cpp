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

#include <pluginlib/class_list_macros.h>

#include <rqt_multiplot/MultiplotWidget.h>

#include "rqt_multiplot/MultiplotPlugin.h"

PLUGINLIB_EXPORT_CLASS(rqt_multiplot::MultiplotPlugin, rqt_gui_cpp::Plugin)

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MultiplotPlugin::MultiplotPlugin() {
  setObjectName("MultiplotPlugin");
}

MultiplotPlugin::~MultiplotPlugin() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MultiplotPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  QStringList argv = context.argv();

  context.addWidget(new MultiplotWidget());  
}

void MultiplotPlugin::shutdownPlugin() {
}

void MultiplotPlugin::saveSettings(qt_gui_cpp::Settings& pluginSettings,
    qt_gui_cpp::Settings& instanceSettings) const {
}

void MultiplotPlugin::restoreSettings(const qt_gui_cpp::Settings&
    pluginSettings, const qt_gui_cpp::Settings& instanceSettings) {
}

}
