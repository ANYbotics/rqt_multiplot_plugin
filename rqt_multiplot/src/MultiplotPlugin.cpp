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

#include <string>
#include <vector>

#include <QByteArray>

#include <boost/program_options.hpp>

#include <pluginlib/class_list_macros.h>

#include <rqt_multiplot/MultiplotWidget.h>

#include "rqt_multiplot/MultiplotPlugin.h"

PLUGINLIB_EXPORT_CLASS(rqt_multiplot::MultiplotPlugin, rqt_gui_cpp::Plugin)

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MultiplotPlugin::MultiplotPlugin() :
  widget_(0) {
  setObjectName("MultiplotPlugin");
}

MultiplotPlugin::~MultiplotPlugin() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MultiplotPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new MultiplotWidget();
  
  context.addWidget(widget_);
  
  parseArguments(context.argv());
}

void MultiplotPlugin::shutdownPlugin() {
}

void MultiplotPlugin::saveSettings(qt_gui_cpp::Settings& pluginSettings,
    qt_gui_cpp::Settings& instanceSettings) const {
  size_t maxHistoryLength = widget_->getMaxConfigHistoryLength();
  QStringList history = widget_->getConfigHistory();
  
  instanceSettings.remove("history");
  
  instanceSettings.setValue("history/max_length",
    (unsigned int)maxHistoryLength);
  
  for (size_t i = 0; i < history.count(); ++i)
    instanceSettings.setValue("history/config_"+QString::number(i),
      history[i]);
}

void MultiplotPlugin::restoreSettings(const qt_gui_cpp::Settings&
    pluginSettings, const qt_gui_cpp::Settings& instanceSettings) {
  size_t maxHistoryLength = widget_->getMaxConfigHistoryLength();
  QStringList history;
  
  maxHistoryLength = instanceSettings.value("history/max_length",
    (unsigned int)maxHistoryLength).toUInt();

  while (instanceSettings.contains("history/config_"+QString::number(
      history.count())))
    history.append(instanceSettings.value("history/config_"+QString::number(
      history.count())).toString());
    
  widget_->setMaxConfigHistoryLength(maxHistoryLength);
  widget_->setConfigHistory(history);
}

void MultiplotPlugin::parseArguments(const QStringList& arguments) {
  size_t argc = arguments.count();
  std::vector<QByteArray> args;
  
  const char *argv[argc+1];
  argv[0] = "rqt_multiplot";

  for (int i = 0; i < argc; ++i) {
    args.push_back(arguments[i].toLocal8Bit());
    argv[i+1] = args[i].constData();
  }

  boost::program_options::variables_map variables;
  boost::program_options::options_description options;
  
  options.add_options()
    ("multiplot-config,c", boost::program_options::value<std::string>(), "");

  try {
    boost::program_options::store(boost::program_options::parse_command_line(
      argc+1, argv, options), variables);
    boost::program_options::notify(variables);

    if (variables.count("multiplot-config"))
      widget_->loadConfig(QString::fromStdString(
        variables["multiplot-config"].as<std::string>()));
  }
  catch (const std::exception& exception) {
    ROS_ERROR("Error parsing command line: %s", exception.what());
  }
}

}
