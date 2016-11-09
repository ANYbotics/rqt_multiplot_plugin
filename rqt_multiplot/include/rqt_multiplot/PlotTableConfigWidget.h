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

#ifndef RQT_MULTIPLOT_PLOT_TABLE_CONFIG_WIDGET_H
#define RQT_MULTIPLOT_PLOT_TABLE_CONFIG_WIDGET_H

#include <QMenu>
#include <QWidget>

#include <rqt_multiplot/PlotTableConfig.h>

namespace Ui {
  class PlotTableConfigWidget;
};

namespace rqt_multiplot {
  class PlotTableWidget;

  class PlotTableConfigWidget :
    public QWidget {
  Q_OBJECT
  public:
    PlotTableConfigWidget(QWidget* parent = 0);
    virtual ~PlotTableConfigWidget();

    void setConfig(PlotTableConfig* config);
    PlotTableConfig* getConfig() const;
    void setPlotTable(PlotTableWidget* plotTable);
    PlotTableWidget* getPlotTableWidget() const;
    void runPlots();

  protected:
    bool eventFilter(QObject* object, QEvent* event);

  private:
    Ui::PlotTableConfigWidget* ui_;

    QMenu* menuImportExport_;

    PlotTableConfig* config_;
    PlotTableWidget* plotTable_;

  private slots:
    void configBackgroundColorChanged(const QColor& color);
    void configForegroundColorChanged(const QColor& color);
    void configNumPlotsChanged(size_t numRows, size_t numColumns);
    void configLinkScaleChanged(bool link);
    void configLinkCursorChanged(bool link);
    void configTrackPointsChanged(bool track);

    void spinBoxRowsValueChanged(int value);
    void spinBoxColumnsValueChanged(int value);

    void checkBoxLinkScaleStateChanged(int state);
    void checkBoxLinkCursorStateChanged(int state);
    void checkBoxTrackPointsStateChanged(int state);

    void pushButtonRunClicked();
    void pushButtonPauseClicked();
    void pushButtonClearClicked();
    void pushButtonImportExportClicked();
    void menuImportBagFileTriggered();
    void menuExportImageFileTriggered();
    void menuExportTextFileTriggered();

    void plotTablePlotPausedChanged();
    void plotTableJobStarted(const QString& toolTip);
    void plotTableJobProgressChanged(double progress);
    void plotTableJobFinished(const QString& toolTip);
    void plotTableJobFailed(const QString& toolTip);
  };
};

#endif
