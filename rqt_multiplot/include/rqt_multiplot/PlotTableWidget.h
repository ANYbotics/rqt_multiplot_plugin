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

#ifndef RQT_MULTIPLOT_PLOT_TABLE_WIDGET_H
#define RQT_MULTIPLOT_PLOT_TABLE_WIDGET_H

#include <QGridLayout>
#include <QVector>
#include <QWidget>

#include <rqt_multiplot/BagReader.h>
#include <rqt_multiplot/BoundingRectangle.h>
#include <rqt_multiplot/MessageSubscriberRegistry.h>
#include <rqt_multiplot/PlotTableConfig.h>

namespace rqt_multiplot {
  class PlotWidget;
  
  class PlotTableWidget :
    public QWidget {
  Q_OBJECT
  public:
    PlotTableWidget(QWidget* parent = 0);
    virtual ~PlotTableWidget();
    
    void setConfig(PlotTableConfig* config);
    PlotTableConfig* getConfig() const;
    size_t getNumRows() const;
    size_t getNumColumns() const;
    PlotWidget* getPlotWidget(size_t row, size_t column) const;
    MessageSubscriberRegistry* getRegistry() const;
    BagReader* getBagReader() const;
    
    void runPlots();
    void pausePlots();
    void clearPlots();

    void requestReplot();
    void forceReplot();
  
    void renderToPixmap(QPixmap& pixmap);
    void writeFormattedCurveAxisTitles(QStringList& formattedAxisTitles);
    void writeFormattedCurveData(QList<QStringList>& formattedData);
    
    void loadFromBagFile(const QString& fileName);
    void saveToImageFile(const QString& fileName);
    void saveToTextFile(const QString& fileName);
    
  signals:
    void plotPausedChanged();
    void jobStarted(const QString& toolTip);
    void jobProgressChanged(double progress);
    void jobFinished(const QString& toolTip);
    void jobFailed(const QString& toolTip);
    
  private:
    QGridLayout* layout_;
    QVector<QVector<PlotWidget*> > plotWidgets_;
    
    PlotTableConfig* config_;
    
    MessageSubscriberRegistry* registry_;
    BagReader* bagReader_;
  
    void updatePlotScale(const BoundingRectangle& bounds, PlotWidget*
      excluded = 0);
    
  private slots:
    void configBackgroundColorChanged(const QColor& color);
    void configForegroundColorChanged(const QColor& color);
    void configNumPlotsChanged(size_t numRows, size_t numColumns);
    void configLinkScaleChanged(bool link);
    void configTrackPointsChanged(bool track);
    
    void plotPreferredScaleChanged(const BoundingRectangle& bounds);
    void plotCurrentScaleChanged(const BoundingRectangle& bounds);
    void plotCursorActiveChanged(bool active);
    void plotCursorCurrentPositionChanged(const QPointF& position);
    void plotPausedChanged(bool paused);    
    void plotStateChanged(int state);
    
    void bagReaderReadingStarted();
    void bagReaderReadingProgressChanged(double progress);
    void bagReaderReadingFinished();
    void bagReaderReadingFailed(const QString& error);
  };
};

#endif
