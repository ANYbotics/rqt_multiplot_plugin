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

#include <ros/ros.h>

#include <ui_plot_widget.h>

namespace rqt_multiplot {

class PlotWidget :
  public QWidget {
Q_OBJECT
public:
  PlotWidget(QWidget* parent = 0, Qt::WindowFlags flags = 0);
  PlotWidget(const PlotWidget& src);

  void setTitle(const QString& title);
  QString getTitle() const;
  
  void init();
  
  void run();
  void pause();
  void clear();
  
protected:
  bool eventFilter(QObject* object, QEvent* event);
   
private:
  Ui::plot_widget ui_;
  
private slots:
  void titleTextChanged(const QString& text);
  
  void runPauseClicked();
  void clearClicked();
  void ejectClicked();
  
  void xTopScaleDivChanged();
  void xBottomScaleDivChanged();
  void yLeftScaleDivChanged();
  void yRightScaleDivChanged();  
};

};

#endif
