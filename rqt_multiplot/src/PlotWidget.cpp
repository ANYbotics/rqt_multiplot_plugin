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
#include <qwt/qwt_scale_widget.h>

#include <ros/package.h>

#include <rqt_multiplot/PlotConfigDialog.h>
#include <rqt_multiplot/PlotConfigWidget.h>

#include <ui_PlotWidget.h>

#include "rqt_multiplot/PlotWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotWidget::PlotWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::PlotWidget()),
  config_(new PlotConfig(this)) {
  ui_->setupUi(this);
  
  ui_->pushButtonRunPause->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/run.png"))));
  ui_->pushButtonClear->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/clear.png"))));
  ui_->pushButtonEdit->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/eject.png"))));
  
  ui_->plot->setAutoFillBackground(true);
  ui_->plot->canvas()->setFrameStyle(QFrame::NoFrame);
  
  ui_->plot->enableAxis(QwtPlot::xTop);
  ui_->plot->enableAxis(QwtPlot::yRight);
  
  ui_->plot->axisScaleDraw(QwtPlot::xTop)->enableComponent(
    QwtAbstractScaleDraw::Labels, false);
  ui_->plot->axisScaleDraw(QwtPlot::yRight)->enableComponent(
    QwtAbstractScaleDraw::Labels, false);
  
  ui_->horizontalSpacerRight->changeSize(
    ui_->plot->axisWidget(QwtPlot::yRight)->width()-5, 20);
      
  connect(config_, SIGNAL(changed()), this, SLOT(configChanged()));
  
  connect(ui_->lineEditTitle, SIGNAL(textChanged(const QString&)), this,
    SLOT(lineEditTitleTextChanged(const QString&)));
  connect(ui_->lineEditTitle, SIGNAL(editingFinished()), this,
    SLOT(lineEditTitleEditingFinished()));
  
  connect(ui_->pushButtonRunPause, SIGNAL(clicked()), this,
    SLOT(pushButtonRunPauseClicked()));
  connect(ui_->pushButtonClear, SIGNAL(clicked()), this,
    SLOT(pushButtonClearClicked()));
  connect(ui_->pushButtonEdit, SIGNAL(clicked()), this,
    SLOT(pushButtonEditClicked()));
  
  connect(ui_->plot->axisWidget(QwtPlot::xTop), 
    SIGNAL(scaleDivChanged()), this, SLOT(plotXTopScaleDivChanged()));
  connect(ui_->plot->axisWidget(QwtPlot::xBottom), 
    SIGNAL(scaleDivChanged()), this, SLOT(plotXBottomScaleDivChanged()));
  connect(ui_->plot->axisWidget(QwtPlot::yLeft), 
    SIGNAL(scaleDivChanged()), this, SLOT(plotYLeftScaleDivChanged()));
  connect(ui_->plot->axisWidget(QwtPlot::yRight), 
    SIGNAL(scaleDivChanged()), this, SLOT(plotYRightScaleDivChanged()));
  
  ui_->plot->axisWidget(QwtPlot::yLeft)->installEventFilter(this);
  ui_->plot->axisWidget(QwtPlot::yRight)->installEventFilter(this);
}

PlotWidget::~PlotWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

PlotConfig* PlotWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotWidget::run() {
}

void PlotWidget::pause() {
}

void PlotWidget::clear() {
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

void PlotWidget::configChanged() {
  ui_->lineEditTitle->setText(config_->getTitle());
}

void PlotWidget::lineEditTitleTextChanged(const QString& text) {
  QFontMetrics fontMetrics(ui_->lineEditTitle->font());
  
  ui_->lineEditTitle->setMinimumWidth(
    std::max(100, fontMetrics.width(text)+10));
}

void PlotWidget::lineEditTitleEditingFinished() {
  config_->setTitle(ui_->lineEditTitle->text());
}

void PlotWidget::pushButtonRunPauseClicked() {
}

void PlotWidget::pushButtonClearClicked() {
  clear();
}

void PlotWidget::pushButtonEditClicked() {
  PlotConfigDialog dialog(this);
  
  dialog.setWindowTitle(config_->getTitle().isEmpty() ?
    "Configure Plot" :
    "Configure \""+config_->getTitle()+"\"");
  dialog.getWidget()->setConfig(*config_);
  
  if (dialog.exec() == QDialog::Accepted)
    *config_ = dialog.getWidget()->getConfig();
}

void PlotWidget::plotXTopScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::xBottom, *ui_->plot->axisScaleDiv(
    QwtPlot::xTop));
}

void PlotWidget::plotXBottomScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::xTop, *ui_->plot->axisScaleDiv(
    QwtPlot::xBottom));
}

void PlotWidget::plotYLeftScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::yRight, *ui_->plot->axisScaleDiv(
    QwtPlot::yLeft));
}

void PlotWidget::plotYRightScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::yLeft, *ui_->plot->axisScaleDiv(
    QwtPlot::yRight));
}

}
