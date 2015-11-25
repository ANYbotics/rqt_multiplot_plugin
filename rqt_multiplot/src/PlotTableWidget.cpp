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

#include "rqt_multiplot/PlotTableWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotTableWidget::PlotTableWidget(QWidget* parent) :
  QWidget(parent),
  layout_(new QGridLayout(this)),
  config_(0) {
  setLayout(layout_);
  setAutoFillBackground(true);
  
  layout_->setHorizontalSpacing(20);
  layout_->setVerticalSpacing(20);
}

PlotTableWidget::~PlotTableWidget() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotTableWidget::setConfig(PlotTableConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(backgroundColorChanged(const QColor&)),
        this, SLOT(configBackgroundColorChanged(const QColor&)));
      disconnect(config_, SIGNAL(numPlotsChanged(size_t, size_t)), this,
        SLOT(configNumPlotsChanged(size_t, size_t)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(backgroundColorChanged(const QColor&)),
        this, SLOT(configBackgroundColorChanged(const QColor&)));
      connect(config, SIGNAL(numPlotsChanged(size_t, size_t)), this,
        SLOT(configNumPlotsChanged(size_t, size_t)));
      
      configBackgroundColorChanged(config->getBackgroundColor());
      configNumPlotsChanged(config->getNumRows(), config->getNumColumns());
    }
  }
}

PlotTableConfig* PlotTableWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotTableWidget::runPlots() {
  for (size_t row = 0; row < layout_->rowCount(); ++row)
    for (size_t column = 0; column < layout_->columnCount(); ++ column)
      static_cast<PlotWidget*>(layout_->itemAtPosition(row, column)->
        widget())->run();
}

void PlotTableWidget::pausePlots() {
  for (size_t row = 0; row < layout_->rowCount(); ++row)
    for (size_t column = 0; column < layout_->columnCount(); ++ column)
      static_cast<PlotWidget*>(layout_->itemAtPosition(row, column)->
        widget())->pause();
}

void PlotTableWidget::clearPlots() {
  for (size_t row = 0; row < layout_->rowCount(); ++row)
    for (size_t column = 0; column < layout_->columnCount(); ++ column)
      static_cast<PlotWidget*>(layout_->itemAtPosition(row, column)->
        widget())->clear();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotTableWidget::configBackgroundColorChanged(const QColor& color) {
  QPalette currentPalette = palette();
  
  currentPalette.setColor(QPalette::Window, color);
  currentPalette.setColor(QPalette::Base, color);
  
  setPalette(currentPalette);
}

void PlotTableWidget::configNumPlotsChanged(size_t numRows, size_t
    numColumns) {
  size_t oldNumRows = plotWidgets_.count();
  size_t oldNumColumns = oldNumRows ? plotWidgets_[0].count() : 0;
  
  if (!numRows || !numColumns) {
    numRows = 0;
    numColumns = 0;
  }
  
  QVector<QVector<PlotWidget* > > plotWidgets(numRows);  
  QGridLayout* layout = new QGridLayout();
  
  layout->setHorizontalSpacing(20);
  layout->setVerticalSpacing(20);
  
  for (size_t row = 0; row < numRows; ++row) {
    plotWidgets[row].resize(numColumns);
    
    for (size_t column = 0; column < numColumns; ++column) {
      if ((row < oldNumRows) && (column < oldNumColumns))
        plotWidgets[row][column] = plotWidgets_[row][column];
      else
        plotWidgets[row][column] = new PlotWidget(this);
      
      if (config_)
        plotWidgets[row][column]->setConfig(config_->getPlotConfig(
          row, column));
      
      layout->addWidget(plotWidgets[row][column], row, column);
    }
  }
  
  for (size_t row = 0; row < oldNumRows; ++row)
    for (size_t column = 0; column < oldNumColumns; ++column)
      if ((row >= numRows) || (column >= numColumns))
        delete plotWidgets_[row][column];
    
  plotWidgets_ = plotWidgets;
  
  delete layout_;
  layout_ = layout;
  setLayout(layout);
}

}
