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

#ifndef RQT_MULTIPLOT_CURVE_STYLE_WIDGET_H
#define RQT_MULTIPLOT_CURVE_STYLE_WIDGET_H

#include <QButtonGroup>
#include <QWidget>

#include <rqt_multiplot/CurveStyle.h>

namespace Ui {
  class CurveStyleWidget;
};

namespace rqt_multiplot {
  class CurveStyleWidget :
    public QWidget {
  Q_OBJECT
  public:
    CurveStyleWidget(QWidget* parent = 0);
    virtual ~CurveStyleWidget();

    void setStyle(CurveStyle* style);
    CurveStyle* getStyle() const;
    
  private:
    Ui::CurveStyleWidget* ui_;
    
    QButtonGroup* buttonGroupSticksOrientation_;
    
    CurveStyle* style_;
    
  private slots:
    void styleTypeChanged(int type);
    
    void styleLinesInterpolateChanged(bool interpolate);
    void styleSticksOrientationChanged(int orientation);
    void styleSticksBaselineChanged(double baseline);
    void styleStepsInvertChanged(bool invert);
    
    void stylePenWidthChanged(size_t width);
    void stylePenStyleChanged(int style);
    void styleRenderAntialiasChanged(bool antialias);
    
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
