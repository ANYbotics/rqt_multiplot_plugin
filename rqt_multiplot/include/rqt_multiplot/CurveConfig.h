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

#ifndef RQT_MULTIPLOT_CURVE_CONFIG_H
#define RQT_MULTIPLOT_CURVE_CONFIG_H

#include <QObject>
#include <QMap>
#include <QString>

#include <rqt_multiplot/CurveAxisConfig.h>
#include <rqt_multiplot/CurveColor.h>

namespace rqt_multiplot {
  class CurveConfig :
    public QObject {
  Q_OBJECT
  public:
    enum Axis {
      X,
      Y
    };
    
    CurveConfig(QObject* parent = 0, const QString& title = "Untitled Curve");
    ~CurveConfig();

    void setTitle(const QString& title);
    const QString& getTitle() const;
    CurveAxisConfig* getAxisConfig(Axis axis) const;
    CurveColor* getColor() const;

    CurveConfig& operator=(const CurveConfig& src);
    
  signals:
    void titleChanged(const QString& title);
    void changed();
    
  private:
    QString title_;
    QMap<Axis, CurveAxisConfig*> axisConfig_;
    CurveColor* color_;
    
  private slots:
    void axisConfigChanged();
    void colorChanged();
  };
};

#endif
