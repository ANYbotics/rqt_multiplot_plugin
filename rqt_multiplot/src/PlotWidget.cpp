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

#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_picker.h>
#include <qwt/qwt_scale_widget.h>

#include <ros/package.h>

#include <rqt_multiplot/PlotConfigDialog.h>
#include <rqt_multiplot/PlotConfigWidget.h>
#include <rqt_multiplot/PlotCursor.h>
#include <rqt_multiplot/PlotCurve.h>
#include <rqt_multiplot/PlotMagnifier.h>
#include <rqt_multiplot/PlotPanner.h>
#include <rqt_multiplot/PlotZoomer.h>

#include <ui_PlotWidget.h>

#include "rqt_multiplot/PlotWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotWidget::PlotWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::PlotWidget()),
  config_(0),
  magnifier_(0),
  paused_(true) {
  qRegisterMetaType<BoundingRectangle>("BoundingRectangle");
  
  ui_->setupUi(this);
  
  runIcon_ = QIcon(QString::fromStdString(ros::package::getPath(
    "rqt_multiplot").append("/resource/16x16/run.png")));
  pauseIcon_ = QIcon(QString::fromStdString(ros::package::getPath(
    "rqt_multiplot").append("/resource/16x16/pause.png")));
  
  ui_->pushButtonRunPause->setIcon(runIcon_);
  ui_->pushButtonClear->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/clear.png"))));
  ui_->pushButtonSetup->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/setup.png"))));
  ui_->pushButtonExport->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/export.png"))));
  
  ui_->plot->setAutoFillBackground(true);
  ui_->plot->canvas()->setFrameStyle(QFrame::NoFrame);

  ui_->plot->enableAxis(QwtPlot::xTop);
  ui_->plot->enableAxis(QwtPlot::yRight);
  
  ui_->plot->setAxisAutoScale(QwtPlot::yLeft, false);
  ui_->plot->setAxisAutoScale(QwtPlot::yRight, false);
  ui_->plot->setAxisAutoScale(QwtPlot::xTop, false);
  ui_->plot->setAxisAutoScale(QwtPlot::xBottom, false);
  
  ui_->plot->axisScaleDraw(QwtPlot::xTop)->enableComponent(
    QwtAbstractScaleDraw::Labels, false);
  ui_->plot->axisScaleDraw(QwtPlot::yRight)->enableComponent(
    QwtAbstractScaleDraw::Labels, false);
  
  ui_->horizontalSpacerRight->changeSize(
    ui_->plot->axisWidget(QwtPlot::yRight)->width()-5, 20);
  
  cursor_ = new PlotCursor(ui_->plot->canvas());
  magnifier_ = new PlotMagnifier(ui_->plot->canvas());
  panner_ = new PlotPanner(ui_->plot->canvas());
  zoomer_ = new PlotZoomer(ui_->plot->canvas());
  zoomer_->setTrackerMode(QwtPicker::AlwaysOff);  
  
  connect(ui_->lineEditTitle, SIGNAL(textChanged(const QString&)), this,
    SLOT(lineEditTitleTextChanged(const QString&)));
  connect(ui_->lineEditTitle, SIGNAL(editingFinished()), this,
    SLOT(lineEditTitleEditingFinished()));
  
  connect(ui_->pushButtonRunPause, SIGNAL(clicked()), this,
    SLOT(pushButtonRunPauseClicked()));
  connect(ui_->pushButtonClear, SIGNAL(clicked()), this,
    SLOT(pushButtonClearClicked()));
  connect(ui_->pushButtonSetup, SIGNAL(clicked()), this,
    SLOT(pushButtonSetupClicked()));
  connect(ui_->pushButtonExport, SIGNAL(clicked()), this,
    SLOT(pushButtonExportClicked()));
  
  connect(ui_->plot->axisWidget(QwtPlot::xBottom), 
    SIGNAL(scaleDivChanged()), this, SLOT(plotXBottomScaleDivChanged()));
  connect(ui_->plot->axisWidget(QwtPlot::yLeft), 
    SIGNAL(scaleDivChanged()), this, SLOT(plotYLeftScaleDivChanged()));
  
  ui_->plot->axisWidget(QwtPlot::yLeft)->installEventFilter(this);
  ui_->plot->axisWidget(QwtPlot::yRight)->installEventFilter(this);  
}

PlotWidget::~PlotWidget() {  
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotWidget::setConfig(PlotConfig* config) {
  if (config != config_) {
    if (config_) {
      disconnect(config_, SIGNAL(titleChanged(const QString&)), this,
        SLOT(configTitleChanged(const QString&)));
      disconnect(config_, SIGNAL(curveAdded(size_t)), this,
        SLOT(configCurveAdded(size_t)));
      disconnect(config_, SIGNAL(curveRemoved(size_t)), this,
        SLOT(configCurveRemoved(size_t)));
      disconnect(config_, SIGNAL(curvesCleared()), this,
        SLOT(configCurvesCleared()));
      
      configCurvesCleared();
    }
    
    config_ = config;
    
    if (config) {
      connect(config, SIGNAL(titleChanged(const QString&)), this,
        SLOT(configTitleChanged(const QString&)));
      connect(config, SIGNAL(curveAdded(size_t)), this,
        SLOT(configCurveAdded(size_t)));
      connect(config, SIGNAL(curveRemoved(size_t)), this,
        SLOT(configCurveRemoved(size_t)));
      connect(config, SIGNAL(curvesCleared()), this,
        SLOT(configCurvesCleared()));
      
      configTitleChanged(config->getTitle());
      
      for (size_t index = 0; index < config->getNumCurves(); ++index)
        configCurveAdded(index);
    }
  }
}

PlotConfig* PlotWidget::getConfig() const {
  return config_;
}

PlotCursor* PlotWidget::getCursor() const {
  return cursor_;
}

BoundingRectangle PlotWidget::getPreferredScale() const {
  BoundingRectangle bounds;
  
  for (size_t index = 0; index < curves_.count(); ++index)
    bounds += curves_[index]->getPreferredScale();
    
  return bounds;
}

void PlotWidget::setCurrentScale(const BoundingRectangle& bounds) {
  if (bounds != getCurrentScale())
    if (bounds.getMaximum().x() >= bounds.getMinimum().x()) {
      ui_->plot->setAxisScale(QwtPlot::xBottom, bounds.getMinimum().x(),
        bounds.getMaximum().x());
    if (bounds.getMaximum().y() >= bounds.getMinimum().y())
      ui_->plot->setAxisScale(QwtPlot::yLeft, bounds.getMinimum().y(),
        bounds.getMaximum().y());
  
    replot();
  }
}

BoundingRectangle PlotWidget::getCurrentScale() const {
  QPointF minimum(ui_->plot->axisScaleDiv(QwtPlot::xBottom)->lowerBound(),
    ui_->plot->axisScaleDiv(QwtPlot::yLeft)->lowerBound());
  QPointF maximum(ui_->plot->axisScaleDiv(QwtPlot::xBottom)->upperBound(),
    ui_->plot->axisScaleDiv(QwtPlot::yLeft)->upperBound());
  
  return BoundingRectangle(minimum, maximum);
}

bool PlotWidget::isPaused() const {
  return paused_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotWidget::run() {
  if (paused_) {
    paused_ = false;
    
    for (size_t index = 0; index < curves_.count(); ++index)
      curves_[index]->run();
    
    ui_->pushButtonRunPause->setIcon(pauseIcon_);
    
    emit pausedChanged(false);
  }
}

void PlotWidget::pause() {
  if (!paused_) {
    for (size_t index = 0; index < curves_.count(); ++index)
      curves_[index]->pause();
      
    paused_ = true;
    
    ui_->pushButtonRunPause->setIcon(runIcon_);
    
    emit pausedChanged(true);
  }
}

void PlotWidget::clear() {
  for (size_t index = 0; index < curves_.count(); ++index)
    curves_[index]->clear();
  
  emit cleared();
}

void PlotWidget::replot() {
  ui_->plot->replot();
}

bool PlotWidget::eventFilter(QObject* object, QEvent* event) {
  if ((object == ui_->plot->axisWidget(QwtPlot::yLeft)) &&
      (event->type() == QEvent::Resize)) {
    ui_->horizontalSpacerLeft->changeSize(
      ui_->plot->axisWidget(QwtPlot::yLeft)->width(), 20);
  }
  else if ((object == ui_->plot->axisWidget(QwtPlot::yRight)) &&
      (event->type() == QEvent::Resize)) {
    ui_->horizontalSpacerRight->changeSize(
      ui_->plot->axisWidget(QwtPlot::yRight)->width()-5, 20);
  }
  
  return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotWidget::configTitleChanged(const QString& title) {
  ui_->lineEditTitle->setText(config_->getTitle());
}

void PlotWidget::configCurveAdded(size_t index) {
  PlotCurve* curve = new PlotCurve(this);

  curve->attach(ui_->plot);
  curve->setConfig(config_->getCurveConfig(index));
  
  connect(curve, SIGNAL(preferredScaleChanged(const BoundingRectangle&)),
    this, SLOT(curvePreferredScaleChanged(const BoundingRectangle&)));
  
  curves_.insert(index, curve);
  
  replot();
}

void PlotWidget::configCurveRemoved(size_t index) {
  curves_[index]->detach();

  delete curves_[index];
  
  curves_.remove(index);
  
  replot();
}

void PlotWidget::configCurvesCleared() {
  for (size_t index = 0; index < curves_.count(); ++index) {
    curves_[index]->detach();
    
    delete curves_[index];
  }
  
  curves_.clear();

  replot();
}

void PlotWidget::curvePreferredScaleChanged(const BoundingRectangle& bounds) {
  BoundingRectangle preferredBounds = getPreferredScale();
  
  zoomer_->setZoomBase(preferredBounds.getRectangle());
  
  emit preferredScaleChanged(preferredBounds);
}

void PlotWidget::lineEditTitleTextChanged(const QString& text) {
  QFontMetrics fontMetrics(ui_->lineEditTitle->font());
  
  ui_->lineEditTitle->setMinimumWidth(
    std::max(100, fontMetrics.width(text)+10));
}

void PlotWidget::lineEditTitleEditingFinished() {
  if (config_)
    config_->setTitle(ui_->lineEditTitle->text());
}

void PlotWidget::pushButtonRunPauseClicked() {
  if (paused_)
    run();
  else
    pause();
}

void PlotWidget::pushButtonClearClicked() {
  clear();
}

void PlotWidget::pushButtonSetupClicked() {
  if (config_) {
    PlotConfigDialog dialog(this);
    
    dialog.setWindowTitle(config_->getTitle().isEmpty() ?
      "Configure Plot" :
      "Configure \""+config_->getTitle()+"\"");
    dialog.getWidget()->setConfig(*config_);
    
    if (dialog.exec() == QDialog::Accepted)
      *config_ = dialog.getWidget()->getConfig();
  }
}

void PlotWidget::pushButtonExportClicked() {
}

void PlotWidget::plotXBottomScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::xTop, *ui_->plot->axisScaleDiv(
    QwtPlot::xBottom));
    
  if (ui_->plot->axisScaleDiv(QwtPlot::yLeft)->isValid() &&
      ui_->plot->axisScaleDiv(QwtPlot::xBottom)->isValid())
    emit currentScaleChanged(getCurrentScale());
}

void PlotWidget::plotYLeftScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::yRight, *ui_->plot->axisScaleDiv(
    QwtPlot::yLeft));
    
  if (ui_->plot->axisScaleDiv(QwtPlot::yLeft)->isValid() &&
      ui_->plot->axisScaleDiv(QwtPlot::xBottom)->isValid())
    emit currentScaleChanged(getCurrentScale());
}

}
