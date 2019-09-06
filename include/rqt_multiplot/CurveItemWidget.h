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

#ifndef RQT_MULTIPLOT_CURVE_ITEM_WIDGET_H
#define RQT_MULTIPLOT_CURVE_ITEM_WIDGET_H

#include <QWidget>

#include <rqt_multiplot/CurveConfig.h>

namespace Ui {
  class CurveItemWidget;
};

namespace rqt_multiplot {
  class CurveItemWidget :
    public QWidget {
  Q_OBJECT
  public:
    CurveItemWidget(QWidget* parent = 0);
    virtual ~CurveItemWidget();

    void setConfig(CurveConfig* config);
    CurveConfig* getConfig() const;
  
  protected:
    bool eventFilter(QObject* object, QEvent* event);
    
  private:
    Ui::CurveItemWidget* ui_;
    
    CurveConfig* config_;
    
  private slots:
    void configTitleChanged(const QString& title);
    void configXAxisConfigChanged();
    void configYAxisConfigChanged();
    void configColorConfigCurrentColorChanged(const QColor& color);
  };
};

#endif
