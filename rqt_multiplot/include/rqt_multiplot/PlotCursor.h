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

#ifndef RQT_MULTIPLOT_PLOT_CURSOR_H
#define RQT_MULTIPLOT_PLOT_CURSOR_H

#include <QColor>
#include <QPoint>
#include <QVector>

#include <qwt/qwt_plot_picker.h>

class QwtPlotCanvas;

namespace rqt_multiplot {
  class PlotCursor :
    public QwtPlotPicker {
  Q_OBJECT
  public:
    PlotCursor(QwtPlotCanvas* canvas);
    ~PlotCursor();

    void setActive(bool active, const QPointF& position = QPointF(0.0, 0.0));
    using QwtPlotPicker::isActive;
    void setCurrentPosition(const QPointF& position);
    const QPointF& getCurrentPosition() const;
    void setTrackPoints(bool track);
    bool arePointsTracked() const;
    bool hasMouseControl() const;

    void update();

    void drawRubberBand(QPainter* painter) const;

  signals:
    void activeChanged(bool active);
    void currentPositionChanged(const QPointF& position);

  protected:
    QRect getTextRect(const QPointF& point, const QFont& font) const;

    QwtText trackerTextF(const QPointF& point) const;

    void begin();
    void move(const QPoint& point);
    bool end(bool ok = true);

    bool eventFilter(QObject* object, QEvent* event);

    void updateDisplay();
    void updateTrackedPoints();

    void drawTrackedPoints(QPainter* painter) const;

  private:
    struct TrackedPoint {
      QPointF position;
      QColor color;
    };

    QPointF currentPosition_;
    QVector<TrackedPoint> trackedPoints_;

    bool trackPoints_;
    bool mouseControl_;

  private slots:
    void plotXAxisScaleDivChanged();
    void plotYAxisScaleDivChanged();
  };
};

#endif
