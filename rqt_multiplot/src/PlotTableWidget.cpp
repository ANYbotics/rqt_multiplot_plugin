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

#include <rqt_multiplot/PlotWidget.h>

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
      disconnect(config_, SIGNAL(foregroundColorChanged(const QColor&)),
        this, SLOT(configForegroundColorChanged(const QColor&)));
      disconnect(config_, SIGNAL(numPlotsChanged(size_t, size_t)), this,
        SLOT(configNumPlotsChanged(size_t, size_t)));
      disconnect(config_, SIGNAL(linkScaleChanged(bool)), this,
        SLOT(configLinkScaleChanged(bool)));
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(backgroundColorChanged(const QColor&)),
        this, SLOT(configBackgroundColorChanged(const QColor&)));
      connect(config, SIGNAL(foregroundColorChanged(const QColor&)),
        this, SLOT(configForegroundColorChanged(const QColor&)));
      connect(config, SIGNAL(numPlotsChanged(size_t, size_t)), this,
        SLOT(configNumPlotsChanged(size_t, size_t)));
      connect(config, SIGNAL(linkScaleChanged(bool)), this,
        SLOT(configLinkScaleChanged(bool)));
      
      configBackgroundColorChanged(config->getBackgroundColor());
      configForegroundColorChanged(config->getForegroundColor());
      configNumPlotsChanged(config->getNumRows(), config->getNumColumns());
      configLinkScaleChanged(config->isScaleLinked());
    }
  }
}

PlotTableConfig* PlotTableWidget::getConfig() const {
  return config_;
}

size_t PlotTableWidget::getNumRows() const {
  return plotWidgets_.count();
}

size_t PlotTableWidget::getNumColumns() const {
  if (!plotWidgets_.isEmpty())
    return plotWidgets_[0].count();
  else
    return 0;
}

PlotWidget* PlotTableWidget::getPlotWidget(size_t row, size_t column) const {
  return plotWidgets_[row][column];
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotTableWidget::runPlots() {
  for (size_t row = 0; row < layout_->rowCount(); ++row) {
    for (size_t column = 0; column < layout_->columnCount(); ++ column) {
      QLayoutItem* item = layout_->itemAtPosition(row, column);
      QWidget* widget = item ? item->widget() : 0;
      
      if (widget)
        static_cast<PlotWidget*>(widget)->run();
    }
  }
}

void PlotTableWidget::pausePlots() {
  for (size_t row = 0; row < layout_->rowCount(); ++row) {
    for (size_t column = 0; column < layout_->columnCount(); ++ column) {
      QLayoutItem* item = layout_->itemAtPosition(row, column);
      QWidget* widget = item ? item->widget() : 0;
      
      if (widget)
        static_cast<PlotWidget*>(widget)->pause();
    }
  }
}

void PlotTableWidget::clearPlots() {
  for (size_t row = 0; row < layout_->rowCount(); ++row) {
    for (size_t column = 0; column < layout_->columnCount(); ++ column) {
      QLayoutItem* item = layout_->itemAtPosition(row, column);
      QWidget* widget = item ? item->widget() : 0;
      
      if (widget)
        static_cast<PlotWidget*>(widget)->clear();
    }
  }
}

void PlotTableWidget::replot() {
  for (size_t row = 0; row < layout_->rowCount(); ++row) {
    for (size_t column = 0; column < layout_->columnCount(); ++ column) {
      QLayoutItem* item = layout_->itemAtPosition(row, column);
      QWidget* widget = item ? item->widget() : 0;
      
      if (widget)
        static_cast<PlotWidget*>(widget)->replot();
    }
  }
}

void PlotTableWidget::updatePlotScale(const BoundingRectangle& bounds) {
  BoundingRectangle validBounds = bounds;
  
  if (!bounds.isValid()) {
    BoundingRectangle currentBounds;
    
    for (size_t row = 0; row < plotWidgets_.count(); ++row)
      for (size_t column = 0; column < plotWidgets_[row].count(); ++column)
        currentBounds += plotWidgets_[row][column]->getCurrentScale();
      
    if (bounds.getMaximum().x() < bounds.getMinimum().x()) {
      validBounds.getMinimum().setX(currentBounds.getMinimum().x());
      validBounds.getMaximum().setX(currentBounds.getMaximum().x());
    }
    
    if (bounds.getMaximum().y() < bounds.getMinimum().y()) {
      validBounds.getMinimum().setY(currentBounds.getMinimum().y());
      validBounds.getMaximum().setY(currentBounds.getMaximum().y());
    }  
  }
  
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++column)
      plotWidgets_[row][column]->setCurrentScale(validBounds);
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotTableWidget::configBackgroundColorChanged(const QColor& color) {
  QPalette currentPalette = palette();
  
  currentPalette.setColor(QPalette::Window, color);
  currentPalette.setColor(QPalette::Base, color);
  
  setPalette(currentPalette);

  replot();
}

void PlotTableWidget::configForegroundColorChanged(const QColor& color) {
  QPalette currentPalette = palette();
  
  currentPalette.setColor(QPalette::WindowText, color);
  currentPalette.setColor(QPalette::Text, color);
  
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
      else {
        plotWidgets[row][column] = new PlotWidget(this);
        
        connect(plotWidgets[row][column], SIGNAL(preferredScaleChanged(
          const BoundingRectangle&)), this, SLOT(plotPreferredScaleChanged(
          const BoundingRectangle&)));
        connect(plotWidgets[row][column], SIGNAL(currentScaleChanged(
          const BoundingRectangle&)), this, SLOT(plotCurrentScaleChanged(
          const BoundingRectangle&)));
        connect(plotWidgets[row][column], SIGNAL(pausedChanged(bool)),
          this, SLOT(plotPausedChanged(bool)));        
      }
      
      plotWidgets[row][column]->setConfig(config_->getPlotConfig(
        row, column));
      
      if (config_->isScaleLinked())
        plotWidgets_[row][column]->setCurrentScale(plotWidgets_[0][0]->
          getCurrentScale());
      
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
  
  emit plotPausedChanged();
}

void PlotTableWidget::configLinkScaleChanged(bool link) {
  if (link) {
    BoundingRectangle bounds;
    
    for (size_t row = 0; row < layout_->rowCount(); ++row)
      for (size_t column = 0; column < layout_->columnCount(); ++ column)
        bounds += plotWidgets_[row][column]->getPreferredScale();
        
    updatePlotScale(bounds);
  }
}

void PlotTableWidget::plotPreferredScaleChanged(const BoundingRectangle&
    bounds) {
  if (config_) {
    if (config_->isScaleLinked()) {
      BoundingRectangle bounds;
      
      for (size_t row = 0; row < layout_->rowCount(); ++row)
        for (size_t column = 0; column < layout_->columnCount(); ++ column)
          bounds += plotWidgets_[row][column]->getPreferredScale();
      
      updatePlotScale(bounds);
    }
    else
      static_cast<PlotWidget*>(sender())->setCurrentScale(bounds);
  }
}

void PlotTableWidget::plotCurrentScaleChanged(const BoundingRectangle&
    bounds) {
  if (config_) {
    if (config_->isScaleLinked())
      updatePlotScale(bounds);
  }
}

void PlotTableWidget::plotPausedChanged(bool paused) {
  emit plotPausedChanged();
}

}
