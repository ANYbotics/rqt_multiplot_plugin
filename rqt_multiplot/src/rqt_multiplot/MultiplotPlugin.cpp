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
#include <QCloseEvent>
#include <QUrl>

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
  widget_(0),
  runAllPlotsOnStart_(false) {
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
  size_t maxConfigHistoryLength = widget_->getMaxConfigHistoryLength();
  QStringList configHistory = widget_->getConfigHistory();

  instanceSettings.remove("history");

  instanceSettings.setValue("history/max_length",
    (unsigned int)maxConfigHistoryLength);

  for (size_t i = 0; i < configHistory.count(); ++i)
    instanceSettings.setValue("history/config_"+QString::number(i),
      configHistory[i]);
}

void MultiplotPlugin::restoreSettings(const qt_gui_cpp::Settings&
    pluginSettings, const qt_gui_cpp::Settings& instanceSettings) {
  size_t maxConfigHistoryLength = widget_->getMaxConfigHistoryLength();

  // the config history may already be populated with one element
  // loaded from the command line, make sure that is kept before
  // appending more.
  QStringList configHistory = widget_->getConfigHistory();

  maxConfigHistoryLength = instanceSettings.value("history/max_length",
    (unsigned int)maxConfigHistoryLength).toUInt();

  while (instanceSettings.contains("history/config_"+QString::
      number(configHistory.count())))
    configHistory.append(instanceSettings.value("history/config_"+
      QString::number(configHistory.count())).toString());

  widget_->setMaxConfigHistoryLength(maxConfigHistoryLength);
  widget_->setConfigHistory(configHistory);
  if (runAllPlotsOnStart_) {
    widget_->runPlots();
  }
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
  options.add_options()
    ("multiplot-bag,b", boost::program_options::value<std::string>(), "");
  options.add_options()
    ("multiplot-run-all,r", boost::program_options::bool_switch(
      &runAllPlotsOnStart_), "");


  try {
    boost::program_options::store(boost::program_options::parse_command_line(
      argc+1, argv, options), variables);
    boost::program_options::notify(variables);

    if (variables.count("multiplot-config")) {
      QUrl url = QUrl::fromUserInput(QString::fromStdString(
        variables["multiplot-config"].as<std::string>()));
      widget_->loadConfig(url.toString());
    }
    if (variables.count("multiplot-bag"))
      widget_->readBag(QString::fromStdString(
        variables["multiplot-bag"].as<std::string>()));
  }
  catch (const std::exception& exception) {
    ROS_ERROR("Error parsing command line: %s", exception.what());
  }
}

}
