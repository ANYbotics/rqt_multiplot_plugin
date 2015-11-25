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

#ifndef RQT_MULTIPLOT_PLOT_CONFIG_H
#define RQT_MULTIPLOT_PLOT_CONFIG_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QVector>

#include <rqt_multiplot/CurveConfig.h>

namespace rqt_multiplot {
  class PlotConfig :
    public QObject {
  Q_OBJECT
  public:
    PlotConfig(QObject* parent = 0, const QString& title = "Untitled Plot");
    ~PlotConfig();

    void setTitle(const QString& title);
    const QString& getTitle() const;
    size_t getNumCurves() const;
    CurveConfig* getCurveConfig(size_t index) const;
    
    CurveConfig* addCurve();
    void removeCurve(CurveConfig* curveConfig);
    void removeCurve(size_t index);
    void clearCurves();
    
    void save(QSettings& settings) const;
    void load(QSettings& settings);
    void reset();
    
    PlotConfig& operator=(const PlotConfig& src);
    
  signals:
    void titleChanged(const QString& title);
    void changed();
    
  private:
    QString title_;
    QVector<CurveConfig*> curveConfig_;
    
  private slots:
    void curveConfigChanged();
  };
};

#endif
