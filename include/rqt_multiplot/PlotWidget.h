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

#ifndef RQT_MULTIPLOT_PLOT_WIDGET_H
#define RQT_MULTIPLOT_PLOT_WIDGET_H

#include <QIcon>
#include <QList>
#include <QMenu>
#include <QRect>
#include <QStringList>
#include <QTimer>
#include <QVector>
#include <QWidget>

#include <rqt_multiplot/BoundingRectangle.h>
#include <rqt_multiplot/MessageBroker.h>
#include <rqt_multiplot/PlotConfig.h>

namespace Ui {
  class PlotWidget;
};

namespace rqt_multiplot {
  class PlotCursor;
  class PlotCurve;
  class PlotLegend;
  class PlotMagnifier;
  class PlotPanner;
  class PlotZoomer;

  class PlotWidget :
    public QWidget {
  Q_OBJECT
  public:
    enum State {
      Normal,
      Maximized
    };

    PlotWidget(QWidget* parent = 0);
    virtual ~PlotWidget();

    void setConfig(PlotConfig* config);
    PlotConfig* getConfig() const;
    void setBroker(MessageBroker* broker);
    MessageBroker* getBroker() const;
    PlotCursor* getCursor() const;
    BoundingRectangle getPreferredScale() const;
    void setCurrentScale(const BoundingRectangle& bounds);
    const BoundingRectangle& getCurrentScale() const;
    bool isPaused() const;
    bool isReplotRequested() const;
    void setState(State state);
    State getState() const;
    void setCanChangeState(bool can);
    bool canChangeState() const;

    void run();
    void pause();
    void clear();

    void requestReplot();
    void forceReplot();

    void renderToPixmap(QPixmap& pixmap, const QRectF& bounds = QRectF());
    void writeFormattedCurveAxisTitles(QStringList& formattedAxisTitles);
    void writeFormattedCurveData(QList<QStringList>& formattedData);

    void saveToImageFile(const QString& fileName);
    void saveToTextFile(const QString& fileName);

  signals:
    void preferredScaleChanged(const BoundingRectangle& bounds);
    void currentScaleChanged(const BoundingRectangle& bounds);
    void pausedChanged(bool paused);
    void stateChanged(int state);
    void cleared();

  protected:
    void dragEnterEvent(QDragEnterEvent* event);
    void dropEvent(QDropEvent* event);

    bool eventFilter(QObject* object, QEvent* event);

  private:
    Ui::PlotWidget* ui_;

    QIcon runIcon_;
    QIcon pauseIcon_;
    QIcon normalIcon_;
    QIcon maximizedIcon_;
    QTimer* timer_;
    QMenu* menuImportExport_;

    PlotConfig* config_;

    MessageBroker* broker_;

    QVector<PlotCurve*> curves_;

    PlotLegend* legend_;
    PlotCursor* cursor_;
    PlotPanner* panner_;
    PlotMagnifier* magnifier_;
    PlotZoomer* zoomer_;

    bool paused_;
    bool rescale_;
    bool replot_;
    State state_;

    BoundingRectangle currentBounds_;

    void updateAxisTitle(PlotAxesConfig::Axis axis);

  private slots:
    void timerTimeout();

    void configTitleChanged(const QString& title);
    void configCurveAdded(size_t index);
    void configCurveRemoved(size_t index);
    void configCurvesCleared();
    void configCurveConfigChanged(size_t index);
    void configXAxisConfigChanged();
    void configYAxisConfigChanged();
    void configLegendConfigChanged();
    void configPlotRateChanged(double rate);

    void curveReplotRequested();

    void lineEditTitleTextChanged(const QString& text);
    void lineEditTitleEditingFinished();

    void pushButtonRunPauseClicked();
    void pushButtonClearClicked();
    void pushButtonSetupClicked();
    void pushButtonImportExportClicked();
    void pushButtonStateClicked();
    void menuExportImageFileTriggered();
    void menuExportTextFileTriggered();

    void plotXBottomScaleDivChanged();
    void plotYLeftScaleDivChanged();
  };
};

#endif
