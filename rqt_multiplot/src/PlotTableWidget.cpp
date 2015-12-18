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

#include <QApplication>
#include <QFile>
#include <QTextStream>

#include <rqt_multiplot/PlotCursor.h>
#include <rqt_multiplot/PlotWidget.h>

#include "rqt_multiplot/PlotTableWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotTableWidget::PlotTableWidget(QWidget* parent) :
  QWidget(parent),
  layout_(new QGridLayout(this)),
  config_(0),
  registry_(new MessageSubscriberRegistry(this)),
  bagReader_(new BagReader(this)) {
  setLayout(layout_);
  setAutoFillBackground(true);
  
  layout_->setHorizontalSpacing(20);
  layout_->setVerticalSpacing(20);
  
  connect(bagReader_, SIGNAL(readingStarted()), this,
    SLOT(bagReaderReadingStarted()));
  connect(bagReader_, SIGNAL(readingProgressChanged(double)), this,
    SLOT(bagReaderReadingProgressChanged(double)));
  connect(bagReader_, SIGNAL(readingFinished()), this,
    SLOT(bagReaderReadingFinished()));
  connect(bagReader_, SIGNAL(readingFailed(const QString&)), this,
    SLOT(bagReaderReadingFailed(const QString&)));
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
      disconnect(config_, SIGNAL(trackPointsChanged(bool)), this,
        SLOT(configTrackPointsChanged(bool)));
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
      connect(config, SIGNAL(trackPointsChanged(bool)), this,
        SLOT(configTrackPointsChanged(bool)));
      
      configBackgroundColorChanged(config->getBackgroundColor());
      configForegroundColorChanged(config->getForegroundColor());
      configNumPlotsChanged(config->getNumRows(), config->getNumColumns());
      configLinkScaleChanged(config->isScaleLinked());
      configTrackPointsChanged(config->arePointsTracked());
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

MessageSubscriberRegistry* PlotTableWidget::getRegistry() const {
  return registry_;
}

BagReader* PlotTableWidget::getBagReader() const {
  return bagReader_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotTableWidget::runPlots() {
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
      plotWidgets_[row][column]->run();
}

void PlotTableWidget::pausePlots() {
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
      plotWidgets_[row][column]->pause();
}

void PlotTableWidget::clearPlots() {
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
      plotWidgets_[row][column]->clear();
}

void PlotTableWidget::requestReplot() {
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
      plotWidgets_[row][column]->requestReplot();
}

void PlotTableWidget::forceReplot() {
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
      plotWidgets_[row][column]->forceReplot();
}

void PlotTableWidget::renderToPixmap(QPixmap& pixmap) {
  size_t numRows = getNumRows();
  size_t numColumns = getNumColumns();
  
  if (numRows && numColumns) {
    double plotWidth = (pixmap.width()-20.0*(numColumns-1.0))/numColumns;
    double plotHeight = (pixmap.height()-20.0*(numRows-1.0))/numRows;
    
    double y = 0.0;
    for (size_t row = 0; row < plotWidgets_.count();
        ++row, y += plotHeight+20.0) {
      double x = 0.0;
      
      for (size_t column = 0; column < plotWidgets_[row].count();
          ++column, x += plotWidth+20.0)
        plotWidgets_[row][column]->renderToPixmap(pixmap,
          QRectF(x, y, plotWidth, plotHeight));
    }
  }
}

void PlotTableWidget::writeFormattedCurveAxisTitles(QStringList&
    formattedAxisTitles) {
  formattedAxisTitles.clear();
  
  for (size_t row = 0; row < plotWidgets_.count(); ++row) {
    for (size_t column = 0; column < plotWidgets_[row].count(); ++column) {
      QStringList formattedCurveAxisTitles;
      
      plotWidgets_[row][column]->writeFormattedCurveAxisTitles(
        formattedCurveAxisTitles);
      
      formattedAxisTitles.append(formattedCurveAxisTitles);
    }
  }
}

void PlotTableWidget::writeFormattedCurveData(QList<QStringList>&
    formattedData) {
  formattedData.clear();
  
  for (size_t row = 0; row < plotWidgets_.count(); ++row) {
    for (size_t column = 0; column < plotWidgets_[row].count(); ++column) {
      QList<QStringList> formattedCurveData;
      
      plotWidgets_[row][column]->writeFormattedCurveData(
        formattedCurveData);
      
      formattedData.append(formattedCurveData);
    }
  }
}

void PlotTableWidget::loadFromBagFile(const QString& fileName) {
  clearPlots();
  
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++column)
      plotWidgets_[row][column]->setBroker(bagReader_);
  
  runPlots();
    
  bagReader_->read(fileName);
}

void PlotTableWidget::saveToImageFile(const QString& fileName) {
  QPixmap pixmap(1280, 1024);

  pixmap.fill(Qt::transparent);  
  renderToPixmap(pixmap);
  
  pixmap.save(fileName, "PNG");
}

void PlotTableWidget::saveToTextFile(const QString& fileName) {
  QFile file(fileName);
  
  if (file.open(QIODevice::WriteOnly)) {
    QStringList formattedAxisTitles;
    QList<QStringList> formattedData;
    
    writeFormattedCurveAxisTitles(formattedAxisTitles);
    writeFormattedCurveData(formattedData);
  
    QTextStream stream(&file);

    stream << "# " << formattedAxisTitles.join(", ") << "\n";
    
    size_t row = 0;
    
    while (true) {
      QStringList dataLineParts;
      bool finished = true;
      
      for (size_t column = 0; column < formattedData.count(); ++column) {
        if (row < formattedData[column].count()) {
          dataLineParts.append(formattedData[column][row]);
          finished &= false;
        }
        else
          dataLineParts.append(QString());
      }

      if (!finished) {
        stream << dataLineParts.join(", ") << "\n";
        row++;
      }
      else
        break;
    }
  }
}

void PlotTableWidget::updatePlotScale(const BoundingRectangle& bounds,
    PlotWidget* excluded) {
  BoundingRectangle validBounds = bounds;
  
  if (!bounds.isValid()) {
    BoundingRectangle currentBounds;
    
    for (size_t row = 0; row < plotWidgets_.count(); ++row)
      for (size_t column = 0; column < plotWidgets_[row].count(); ++column)
        currentBounds += plotWidgets_[row][column]->getCurrentScale();
      
    if (bounds.getMaximum().x() <= bounds.getMinimum().x()) {
      validBounds.getMinimum().setX(currentBounds.getMinimum().x());
      validBounds.getMaximum().setX(currentBounds.getMaximum().x());
    }
    
    if (bounds.getMaximum().y() <= bounds.getMinimum().y()) {
      validBounds.getMinimum().setY(currentBounds.getMinimum().y());
      validBounds.getMaximum().setY(currentBounds.getMaximum().y());
    }  
  }
  
  for (size_t row = 0; row < plotWidgets_.count(); ++row) {
    for (size_t column = 0; column < plotWidgets_[row].count(); ++column) {
      if (excluded != plotWidgets_[row][column])
        plotWidgets_[row][column]->setCurrentScale(validBounds);
    }
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotTableWidget::configBackgroundColorChanged(const QColor& color) {
  QPalette currentPalette = palette();
  
  currentPalette.setColor(QPalette::Window, color);
  currentPalette.setColor(QPalette::Base, color);
  
  setPalette(currentPalette);

  forceReplot();
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
        connect(plotWidgets[row][column]->getCursor(), SIGNAL(
          activeChanged(bool)), this, SLOT(plotCursorActiveChanged(bool)));
        connect(plotWidgets[row][column]->getCursor(), SIGNAL(
          currentPositionChanged(const QPointF&)), this, SLOT(
          plotCursorCurrentPositionChanged(const QPointF&)));
        connect(plotWidgets[row][column], SIGNAL(pausedChanged(bool)),
          this, SLOT(plotPausedChanged(bool)));        
        connect(plotWidgets[row][column], SIGNAL(stateChanged(int)),
          this, SLOT(plotStateChanged(int)));        
      }
      
      plotWidgets[row][column]->setConfig(config_->getPlotConfig(
        row, column));
      plotWidgets[row][column]->setBroker(registry_);
      
      if (config_->isScaleLinked())
        plotWidgets[row][column]->setCurrentScale(plotWidgets[0][0]->
          getCurrentScale());
        
      plotWidgets[row][column]->getCursor()->setTrackPoints(
        config_->arePointsTracked());
      
      layout->addWidget(plotWidgets[row][column], row, column);
    }
  }
  
  if ((numRows == 1) && (numColumns == 1))
    plotWidgets[0][0]->setCanChangeState(false);
  else 
    plotWidgets[0][0]->setCanChangeState(true);
  
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
    
    for (size_t row = 0; row < plotWidgets_.count(); ++row)
      for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
        bounds += plotWidgets_[row][column]->getPreferredScale();
        
    updatePlotScale(bounds);
  }
}

void PlotTableWidget::configTrackPointsChanged(bool track) {
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
      plotWidgets_[row][column]->getCursor()->setTrackPoints(track);
}

void PlotTableWidget::bagReaderReadingStarted() {
  emit jobStarted("Reading bag from [file://"+
    bagReader_->getFileName()+"]...");
}

void PlotTableWidget::bagReaderReadingProgressChanged(double progress) {
  emit jobProgressChanged(progress);
}

void PlotTableWidget::bagReaderReadingFinished() {
  pausePlots();
  
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++column)
      plotWidgets_[row][column]->setBroker(registry_);
    
  emit jobFinished("Read bag from [file://"+
    bagReader_->getFileName()+"]");
}

void PlotTableWidget::bagReaderReadingFailed(const QString& error) {
  pausePlots();
  
  for (size_t row = 0; row < plotWidgets_.count(); ++row)
    for (size_t column = 0; column < plotWidgets_[row].count(); ++column)
      plotWidgets_[row][column]->setBroker(registry_);
    
  emit jobFailed("Failed to read bag from [file://"+
    bagReader_->getFileName()+"]");
}

void PlotTableWidget::plotPreferredScaleChanged(const BoundingRectangle&
    bounds) {
  if (config_) {
    if (config_->isScaleLinked()) {
      BoundingRectangle bounds;
      
      for (size_t row = 0; row < plotWidgets_.count(); ++row)
        for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
          bounds += plotWidgets_[row][column]->getPreferredScale();

      updatePlotScale(bounds);
    }
    else
      static_cast<PlotWidget*>(sender())->setCurrentScale(bounds);
  }
}

void PlotTableWidget::plotCurrentScaleChanged(const BoundingRectangle&
    bounds) {
  if (config_ && config_->isScaleLinked())
    updatePlotScale(bounds, static_cast<PlotWidget*>(sender()));
}

void PlotTableWidget::plotCursorActiveChanged(bool active) {
  if (config_ && config_->isCursorLinked()) {
    for (size_t row = 0; row < plotWidgets_.count(); ++row)
      for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
        if (sender() != plotWidgets_[row][column])
          plotWidgets_[row][column]->getCursor()->setActive(active);
  }
}

void PlotTableWidget::plotCursorCurrentPositionChanged(const QPointF&
    position) {
  if (config_ && config_->isCursorLinked()) {
    for (size_t row = 0; row < plotWidgets_.count(); ++row)
      for (size_t column = 0; column < plotWidgets_[row].count(); ++ column)
        if (sender() != plotWidgets_[row][column])
          plotWidgets_[row][column]->getCursor()->setCurrentPosition(
            position);
  }
}

void PlotTableWidget::plotPausedChanged(bool paused) {
  emit plotPausedChanged();
}

void PlotTableWidget::plotStateChanged(int state) {
  for (size_t row = 0; row < plotWidgets_.count(); ++row) {
    for (size_t column = 0; column < plotWidgets_[row].count(); ++ column) {
      if (state == PlotWidget::Maximized) {
        if (sender() != plotWidgets_[row][column])
          plotWidgets_[row][column]->hide();
      }
      else if (state == PlotWidget::Normal)
        plotWidgets_[row][column]->show();
    }
  }
}

}
