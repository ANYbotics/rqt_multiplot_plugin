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

#ifndef RQT_MULTIPLOT_CURVE_COLOR_WIDGET_H
#define RQT_MULTIPLOT_CURVE_COLOR_WIDGET_H

#include <QWidget>

#include <rqt_multiplot/CurveColor.h>

namespace Ui {
  class CurveColorWidget;
};

namespace rqt_multiplot {
  class CurveColorWidget :
    public QWidget {
  Q_OBJECT
  public:
    CurveColorWidget(QWidget* parent = 0);
    virtual ~CurveColorWidget();
    
    void setColor(CurveColor* color);
    CurveColor* getColor() const;
  
  signals:
    void currentColorChanged(const QColor& color);  
    
  protected:
    bool eventFilter(QObject* object, QEvent* event);
    
  private:
    Ui::CurveColorWidget* ui_;
    
    CurveColor* color_;
    
  private slots:
    void colorTypeChanged(int type);
    void colorCurrentColorChanged(const QColor& color);
    
    void checkBoxAutoStateChanged(int state);
  };
};

#endif
