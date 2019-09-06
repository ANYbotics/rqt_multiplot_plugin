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

#ifndef RQT_MULTIPLOT_CURVE_DATA_CONFIG_WIDGET_H
#define RQT_MULTIPLOT_CURVE_DATA_CONFIG_WIDGET_H

#include <QWidget>

#include <rqt_multiplot/CurveDataConfig.h>

namespace Ui {
  class CurveDataConfigWidget;
};

namespace rqt_multiplot {
  class CurveDataConfigWidget :
    public QWidget {
  Q_OBJECT
  public:
    CurveDataConfigWidget(QWidget* parent = 0);
    virtual ~CurveDataConfigWidget();
    
    void setConfig(CurveDataConfig* range);
    CurveDataConfig* getConfig() const;
    
  private:
    Ui::CurveDataConfigWidget* ui_;
    
    CurveDataConfig* config_;
    
  private slots:
    void configTypeChanged(int type);
    void configCircularBufferCapacityChanged(size_t capacity);
    void configTimeFrameLengthChanged(double length);
    
    void radioButtonVectorToggled(bool checked);
    void radioButtonListToggled(bool checked);
    void radioButtonCircularBufferToggled(bool checked);
    void radioButtonTimeFrameToggled(bool checked);
    
    void spinBoxCircularBufferCapacityValueChanged(int value);
    void doubleSpinBoxTimeFrameLengthValueChanged(double value);
  };
};

#endif
