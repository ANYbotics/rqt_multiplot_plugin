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

#include <rqt_multiplot/PlotTableConfig.h>
#include <rqt_multiplot/PlotWidget.h>

namespace rqt_multiplot {
  class PlotTableWidget :
    public QWidget {
  Q_OBJECT
  public:
    PlotTableWidget(QWidget* parent = 0);
    virtual ~PlotTableWidget();
    
    void setConfig(PlotTableConfig* config);
    PlotTableConfig* getConfig() const;
    
    void runPlots();
    void pausePlots();
    void clearPlots();

    void replot();
  private:
    QGridLayout* layout_;
    QVector<QVector<PlotWidget*> > plotWidgets_;
    
    PlotTableConfig* config_;
  
  private slots:
    void configBackgroundColorChanged(const QColor& color);
    void configForegroundColorChanged(const QColor& color);
    void configNumPlotsChanged(size_t numRows, size_t numColumns);
  };
};

#endif
