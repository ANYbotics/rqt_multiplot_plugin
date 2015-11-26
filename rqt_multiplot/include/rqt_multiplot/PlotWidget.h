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

#ifndef RQT_MULTIPLOT_PLOT_WIDGET_H
#define RQT_MULTIPLOT_PLOT_WIDGET_H

#include <QWidget>

#include <rqt_multiplot/PlotConfig.h>

namespace Ui {
  class PlotWidget;
};

namespace rqt_multiplot {
  class PlotWidget :
    public QWidget {
  Q_OBJECT
  public:
    PlotWidget(QWidget* parent = 0);
    virtual ~PlotWidget();

    void setConfig(PlotConfig* config);
    PlotConfig* getConfig() const;
    
    void run();
    void pause();
    void clear();
    
    void replot();
    
  protected:
    bool eventFilter(QObject* object, QEvent* event);
    
  private:
    Ui::PlotWidget* ui_;
    
    PlotConfig* config_;
    
    void init();
    
  private slots:
    void configTitleChanged(const QString& title);
    
    void lineEditTitleTextChanged(const QString& text);
    void lineEditTitleEditingFinished();
    
    void pushButtonRunPauseClicked();
    void pushButtonClearClicked();
    void pushButtonSetupClicked();
    void pushButtonExportClicked();
    
    void plotXTopScaleDivChanged();
    void plotXBottomScaleDivChanged();
    void plotYLeftScaleDivChanged();
    void plotYRightScaleDivChanged();  
  };
};

#endif
