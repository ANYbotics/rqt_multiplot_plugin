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

#ifndef RQT_MULTIPLOT_CURVE_AXIS_SCALE_CONFIG_WIDGET_H
#define RQT_MULTIPLOT_CURVE_AXIS_SCALE_CONFIG_WIDGET_H

#include <QWidget>

#include <rqt_multiplot/CurveAxisScaleConfig.h>

namespace Ui {
  class CurveAxisScaleConfigWidget;
};

namespace rqt_multiplot {
  class CurveAxisScaleConfigWidget :
    public QWidget {
  Q_OBJECT
  public:
    CurveAxisScaleConfigWidget(QWidget* parent = 0);
    virtual ~CurveAxisScaleConfigWidget();
    
    void setConfig(CurveAxisScaleConfig* config);
    CurveAxisScaleConfig* getConfig() const;
    
  private:
    Ui::CurveAxisScaleConfigWidget* ui_;
    
    CurveAxisScaleConfig* config_;
    
  private slots:
    void configTypeChanged(int type);
    void configAbsoluteMinimumChanged(double minimum);
    void configAbsoluteMaximumChanged(double maximum);
    void configRelativeMinimumChanged(double minimum);
    void configRelativeMaximumChanged(double maximum);
    
    void radioButtonAbsoluteToggled(bool checked);
    void radioButtonRelativeToggled(bool checked);
    void radioButtonAutoToggled(bool checked);
    
    void lineEditAbsoluteMinimumEditingFinished();
    void lineEditAbsoluteMaximumEditingFinished();
    void lineEditRelativeMinimumEditingFinished();
    void lineEditRelativeMaximumEditingFinished();
  };
};

#endif
