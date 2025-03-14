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

#ifndef RQT_MULTIPLOT_PLOT_LEGEND_CONFIG_H
#define RQT_MULTIPLOT_PLOT_LEGEND_CONFIG_H

#include <rqt_multiplot/Config.h>

namespace rqt_multiplot {
class PlotLegendConfig : public Config {
  Q_OBJECT
 public:
  explicit PlotLegendConfig(QObject* parent = nullptr, bool visible = true);
  ~PlotLegendConfig() override;

  void setVisible(bool visible);
  bool isVisible() const;

  void save(QSettings& settings) const override;
  void load(QSettings& settings) override;
  void reset() override;

  void write(QDataStream& stream) const override;
  void read(QDataStream& stream) override;

  PlotLegendConfig& operator=(const PlotLegendConfig& src);

 signals:
  void visibleChanged(bool visible);

 private:
  bool visible_;
};
}  // namespace rqt_multiplot

#endif
