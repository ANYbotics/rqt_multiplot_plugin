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

#ifndef RQT_MULTIPLOT_CURVE_LIST_WIDGET_H
#define RQT_MULTIPLOT_CURVE_LIST_WIDGET_H

#include <QListWidget>

#include <rqt_multiplot/CurveConfig.h>
#include <rqt_multiplot/CurveItemWidget.h>

namespace rqt_multiplot {
  class CurveListWidget :
    public QListWidget {
  Q_OBJECT
  public:
    CurveListWidget(QWidget* parent = 0);
    virtual ~CurveListWidget();
    
    size_t getNumCurves() const;
    CurveItemWidget* getCurveItem(size_t index) const;
    
    void addCurve(CurveConfig* config);
    void removeCurve(size_t index);
    
  signals:
    void curveAdded(size_t index);
    void curveConfigChanged(size_t index);
    void curveRemoved(size_t index);
    
  protected:
    void keyPressEvent(QKeyEvent* event);
  };
};

#endif
