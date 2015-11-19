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

#ifndef RQT_MULTIPLOT_CURVE_AXIS_RANGE_WIDGET_H
#define RQT_MULTIPLOT_CURVE_AXIS_RANGE_WIDGET_H

#include <QWidget>

#include <rqt_multiplot/CurveAxisRange.h>

namespace Ui {
  class CurveAxisRangeWidget;
};

namespace rqt_multiplot {
  class CurveAxisRangeWidget :
    public QWidget {
  Q_OBJECT
  public:
    CurveAxisRangeWidget(QWidget* parent = 0);
    virtual ~CurveAxisRangeWidget();
    
    void setRange(CurveAxisRange* range);
    CurveAxisRange* getRange() const;
    
  private:
    Ui::CurveAxisRangeWidget* ui_;
    
    CurveAxisRange* range_;
    
  private slots:
    void rangeTypeChanged(int type);
    void rangeFixedMinimumChanged(double minimum);
    void rangeFixedMaximumChanged(double maximum);
    void rangeWindowSizeChanged(double size);
    
    void radioButtonFixedToggled(bool checked);
    void radioButtonWindowToggled(bool checked);
    void radioButtonAutoToggled(bool checked);
    
    void doubleSpinBoxFixedMinimumValueChanged(double value);
    void doubleSpinBoxFixedMaximumValueChanged(double value);
    void doubleSpinBoxWindowSizeValueChanged(double value);
  };
};

#endif
