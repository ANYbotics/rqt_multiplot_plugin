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

#include <QString>
#include <QVector>

#include <rqt_multiplot/Config.h>
#include <rqt_multiplot/CurveConfig.h>
#include <rqt_multiplot/PlotAxesConfig.h>
#include <rqt_multiplot/PlotLegendConfig.h>

namespace rqt_multiplot {
  class PlotConfig :
    public Config {
  Q_OBJECT
  public:
    PlotConfig(QObject* parent = 0, const QString& title = "Untitled Plot",
      double plotRate = 30.0);
    ~PlotConfig();

    void setTitle(const QString& title);
    const QString& getTitle() const;
    void setNumCurves(size_t numCurves);
    size_t getNumCurves() const;
    CurveConfig* getCurveConfig(size_t index) const;
    PlotAxesConfig* getAxesConfig() const;
    PlotLegendConfig* getLegendConfig() const;
    void setPlotRate(double rate);
    double getPlotRate() const;
    
    CurveConfig* addCurve();
    void removeCurve(CurveConfig* curveConfig);
    void removeCurve(size_t index);
    void clearCurves();
    
    QVector<CurveConfig*> findCurves(const QString& title) const;
    
    void save(QSettings& settings) const;
    void load(QSettings& settings);
    void reset();
    
    void write(QDataStream& stream) const;
    void read(QDataStream& stream);
    
    PlotConfig& operator=(const PlotConfig& src);
    
  signals:
    void titleChanged(const QString& title);
    void curveAdded(size_t index);
    void curveRemoved(size_t index);
    void curvesCleared();
    void curveConfigChanged(size_t index);
    void plotRateChanged(double rate);
    
  private:
    QString title_;
    QVector<CurveConfig*> curveConfig_;
    PlotAxesConfig* axesConfig_;
    PlotLegendConfig* legendConfig_;
    double plotRate_;
    
  private slots:
    void curveConfigChanged();
    void curveConfigDestroyed();
    void axesConfigChanged();
    void legendConfigChanged();
  };
};

#endif
