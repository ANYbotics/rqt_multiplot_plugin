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

#ifndef RQT_MULTIPLOT_CURVE_STYLE_CONFIG_WIDGET_H
#define RQT_MULTIPLOT_CURVE_STYLE_CONFIG_WIDGET_H

#include <QButtonGroup>
#include <QWidget>

#include <rqt_multiplot/CurveStyleConfig.h>

namespace Ui {
  class CurveStyleConfigWidget;
};

namespace rqt_multiplot {
  class CurveStyleConfigWidget :
    public QWidget {
  Q_OBJECT
  public:
    CurveStyleConfigWidget(QWidget* parent = 0);
    virtual ~CurveStyleConfigWidget();

    void setConfig(CurveStyleConfig* config);
    CurveStyleConfig* getConfig() const;
    
  private:
    Ui::CurveStyleConfigWidget* ui_;
    
    QButtonGroup* buttonGroupSticksOrientation_;
    
    CurveStyleConfig* config_;
    
  private slots:
    void configTypeChanged(int type);
    
    void configLinesInterpolateChanged(bool interpolate);
    void configSticksOrientationChanged(int orientation);
    void configSticksBaselineChanged(double baseline);
    void configStepsInvertChanged(bool invert);
    
    void configPenWidthChanged(size_t width);
    void configPenStyleChanged(int style);
    void configRenderAntialiasChanged(bool antialias);
    
    void radioButtonLinesToggled(bool checked);
    void radioButtonSticksToggled(bool checked);
    void radioButtonStepsToggled(bool checked);
    void radioButtonPointsToggled(bool checked);
    
    void checkBoxLinesInterpolateStateChanged(int state);
    void radioButtonSticksOrientationHorizontalToggled(bool checked);
    void radioButtonSticksOrientationVerticalToggled(bool checked);
    void lineEditSticksBaselineEditingFinished();    
    void checkBoxStepsInvertStateChanged(int state);
    
    void spinBoxPenWidthValueChanged(int value);
    void comboBoxPenStyleCurrentStyleChanged(int style);
    void checkBoxRenderAntialiasStateChanged(int state);
  };
};

#endif
