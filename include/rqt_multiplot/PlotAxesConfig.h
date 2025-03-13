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

#ifndef RQT_MULTIPLOT_PLOT_AXES_CONFIG_H
#define RQT_MULTIPLOT_PLOT_AXES_CONFIG_H

#include <rqt_multiplot/Config.h>
#include <rqt_multiplot/PlotAxisConfig.h>

namespace rqt_multiplot {
class PlotAxesConfig : public Config {
  Q_OBJECT
 public:
  enum Axis { X, Y };

  explicit PlotAxesConfig(QObject* parent = nullptr);
  ~PlotAxesConfig() override;

  PlotAxisConfig* getAxisConfig(Axis axis) const;

  void save(QSettings& settings) const override;
  void load(QSettings& settings) override;
  void reset() override;

  void write(QDataStream& stream) const override;
  void read(QDataStream& stream) override;

  PlotAxesConfig& operator=(const PlotAxesConfig& src);

 private:
  QMap<Axis, PlotAxisConfig*> axisConfig_;

 private slots:
  void axisConfigChanged();
};
}  // namespace rqt_multiplot

#endif
