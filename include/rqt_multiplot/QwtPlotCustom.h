/******************************************************************************
 * Copyright (C) 2015 by Ralf Kaestner, Samuel Bachmann                       *
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

#pragma once

#include <qwt/qwt_plot.h>

namespace rqt_multiplot {

class QwtPlotCustom : public QwtPlot {
  Q_OBJECT
 public:
  explicit QwtPlotCustom(QWidget* = nullptr);

  explicit QwtPlotCustom(const QwtText& title, QWidget* p = nullptr);

  QSize sizeHint() const override;

  QSize minimumSizeHint() const override;
};

}  // namespace rqt_multiplot
