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

#ifndef RQT_MULTIPLOT_PLOT_AXIS_CONFIG_WIDGET_H
#define RQT_MULTIPLOT_PLOT_AXIS_CONFIG_WIDGET_H

#include <QWidget>

#include <rqt_multiplot/PlotAxisConfig.h>

namespace Ui {
  class PlotAxisConfigWidget;
};

namespace rqt_multiplot {
  class PlotAxisConfigWidget :
    public QWidget {
  Q_OBJECT
  public:
    PlotAxisConfigWidget(QWidget* parent = 0);
    virtual ~PlotAxisConfigWidget();
    
    void setConfig(PlotAxisConfig* range);
    PlotAxisConfig* getConfig() const;
    
  private:
    Ui::PlotAxisConfigWidget* ui_;
    
    PlotAxisConfig* config_;
    
  private slots:
    void configTitleTypeChanged(int type);
    void configCustomTitleChanged(const QString& title);
    void configTitleVisibleChanged(bool visible);
    
    void checkBoxTitleAutoStateChanged(int state);
    void lineEditTitleEditingFinished();
    void checkBoxTitleVisibleStateChanged(int state);
  };
};

#endif
