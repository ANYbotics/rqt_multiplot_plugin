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

#include <ros/package.h>

#include <rqt_multiplot/PlotWidget.h>

#include <ui_MultiplotPlugin.h>

#include "rqt_multiplot/MultiplotPlugin.h"

PLUGINLIB_EXPORT_CLASS(rqt_multiplot::MultiplotPlugin, rqt_gui_cpp::Plugin)

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MultiplotPlugin::MultiplotPlugin() :
  rqt_gui_cpp::Plugin(),
  ui_(0),
  widget_(0),
  layout_(0) {
  setObjectName("MultiplotPlugin");
}

MultiplotPlugin::~MultiplotPlugin() {
  if (widget_)
    delete widget_;
  if (ui_)
    delete ui_;
  if (layout_)
    delete layout_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MultiplotPlugin::setNumPlots(size_t numRows, size_t numColumns) {
  if (layout_)
    delete layout_;
  
  layout_ = new QGridLayout();
  ui_->framePlots->setLayout(layout_);

  size_t oldNumRows = plots_.size();
  size_t oldNumColumns = plots_.empty() ? 0 : plots_[0].size();
  
  plots_.resize(numRows);
  
  for (size_t row = 0; row < numRows; ++row) {
    plots_[row].resize(numColumns);
    
    for (size_t col = 0; col < numColumns; ++ col) {
      if ((row < oldNumRows) && (col < oldNumColumns)) {
        plots_[row][col] = PlotWidgetPtr(new PlotWidget(*plots_[row][col]));
      }
      else {
        plots_[row][col] = PlotWidgetPtr(new PlotWidget(widget_));
        plots_[row][col]->setTitle("Untitled Plot");
      }
      
      layout_->addWidget(plots_[row][col].get(), row, col);
    }
  }
  
  setBackgroundColor(Qt::white);
}

void MultiplotPlugin::setBackgroundColor(const QColor& color) {
  QPalette palette;
  palette.setColor(QPalette::Window, color);
  
  for (size_t row = 0; row < plots_.size(); ++row)
    for (size_t col = 0; col < plots_[row].size(); ++ col)
      plots_[row][col]->setPalette(palette);

  ui_->framePlots->setPalette(palette);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MultiplotPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  QStringList argv = context.argv();

  ui_ = new Ui::MultiplotPlugin();
  widget_ = new QWidget();
  
  ui_->setupUi(widget_);
  context.addWidget(widget_);
  
  ui_->pushButtonRun->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/run.png"))));
  ui_->pushButtonPause->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/pause.png"))));
  ui_->pushButtonClear->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/clear.png"))));
  
  ui_->framePlots->setAutoFillBackground(true);

  connect(ui_->spinBoxRows, SIGNAL(valueChanged(int)), this,
    SLOT(spinBoxRowsValueChanged(int)));
  connect(ui_->spinBoxColumns, SIGNAL(valueChanged(int)), this,
    SLOT(spinBoxColumnsValueChanged(int)));

  connect(ui_->pushButtonRun, SIGNAL(clicked()), this, SLOT(runClicked()));
  connect(ui_->pushButtonPause, SIGNAL(clicked()), this, SLOT(pauseClicked()));
  connect(ui_->pushButtonClear, SIGNAL(clicked()), this, SLOT(clearClicked()));
  
  setNumPlots(1, 1);
}

void MultiplotPlugin::shutdownPlugin() {
}

void MultiplotPlugin::saveSettings(qt_gui_cpp::Settings& pluginSettings,
    qt_gui_cpp::Settings& instanceSettings) const {
}

void MultiplotPlugin::restoreSettings(const qt_gui_cpp::Settings&
    pluginSettings, const qt_gui_cpp::Settings& instanceSettings) {
}

void MultiplotPlugin::run() {
  for (size_t row = 0; row < plots_.size(); ++row)
    for (size_t col = 0; col < plots_[row].size(); ++ col)
      plots_[row][col]->run();
}

void MultiplotPlugin::pause() {
  for (size_t row = 0; row < plots_.size(); ++row)
    for (size_t col = 0; col < plots_[row].size(); ++ col)
      plots_[row][col]->pause();
}

void MultiplotPlugin::clear() {
  for (size_t row = 0; row < plots_.size(); ++row)
    for (size_t col = 0; col < plots_[row].size(); ++ col)
      plots_[row][col]->clear();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MultiplotPlugin::spinBoxRowsValueChanged(int value) {
  setNumPlots(ui_->spinBoxRows->value(), ui_->spinBoxColumns->value());
}

void MultiplotPlugin::spinBoxColumnsValueChanged(int value) {
  setNumPlots(ui_->spinBoxRows->value(), ui_->spinBoxColumns->value());
}

void MultiplotPlugin::runClicked() {
  run();
}

void MultiplotPlugin::pauseClicked() {
  pause();
}

void MultiplotPlugin::clearClicked() {
  clear();
}

}
