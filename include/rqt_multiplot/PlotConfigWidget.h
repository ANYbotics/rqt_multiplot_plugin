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

#ifndef RQT_MULTIPLOT_PLOT_CONFIG_WIDGET_H
#define RQT_MULTIPLOT_PLOT_CONFIG_WIDGET_H

#include <QListWidgetItem>
#include <QWidget>

#include <rqt_multiplot/PlotConfig.h>

namespace Ui {
  class PlotConfigWidget;
};

namespace rqt_multiplot {
  class PlotConfigWidget :
    public QWidget {
  Q_OBJECT
  public:
    PlotConfigWidget(QWidget* parent = 0);
    virtual ~PlotConfigWidget();

    void setConfig(const PlotConfig& config);
    const PlotConfig& getConfig() const;
    
    void copySelectedCurves();
    void pasteCurves();

    bool eventFilter(QObject* object, QEvent* event);
    
  private:
    Ui::PlotConfigWidget* ui_;
    
    PlotConfig* config_;
  
  private slots:
    void configTitleChanged(const QString& title);
    void configPlotRateChanged(double rate);

    void lineEditTitleEditingFinished();
    
    void pushButtonAddCurveClicked();
    void pushButtonEditCurveClicked();
    void pushButtonRemoveCurvesClicked();
    
    void pushButtonCopyCurvesClicked();
    void pushButtonPasteCurvesClicked();
    
    void curveListWidgetItemSelectionChanged();
    void curveListWidgetItemDoubleClicked(QListWidgetItem* item);
    
    void doubleSpinBoxPlotRateValueChanged(double value);
    
    void clipboardDataChanged();
  };
};

#endif
