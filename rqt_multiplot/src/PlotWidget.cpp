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
  ui_(new Ui::PlotWidget()) {
  init();
}

PlotWidget::PlotWidget(const PlotWidget& src) :
  QWidget(src.parentWidget(), src.windowFlags()),
  ui_(new Ui::PlotWidget()) {
  init();
  
  setTitle(src.getTitle());
}

PlotWidget::~PlotWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotWidget::setTitle(const QString& title) {
  ui_->lineEditTitle->setText(title);
}

QString PlotWidget::getTitle() const {
  return ui_->lineEditTitle->text();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PlotWidget::init() {
  ui_->setupUi(this);  
  
  ui_->pushButtonRunPause->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/run.png"))));
  ui_->pushButtonClear->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/clear.png"))));
  ui_->pushButtonEject->setIcon(
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
      
  connect(ui_->lineEditTitle, SIGNAL(textChanged(const QString&)), this,
    SLOT(titleTextChanged(const QString&)));
  
  connect(ui_->pushButtonRunPause, SIGNAL(clicked()), this,
    SLOT(runPauseClicked()));
  connect(ui_->pushButtonClear, SIGNAL(clicked()), this, SLOT(clearClicked()));
  connect(ui_->pushButtonEject, SIGNAL(clicked()), this, SLOT(ejectClicked()));
  
  connect(ui_->plot->axisWidget(QwtPlot::xTop), 
    SIGNAL(scaleDivChanged()), this, SLOT(xTopScaleDivChanged()));
  connect(ui_->plot->axisWidget(QwtPlot::xBottom), 
    SIGNAL(scaleDivChanged()), this, SLOT(xBottomScaleDivChanged()));
  connect(ui_->plot->axisWidget(QwtPlot::yLeft), 
    SIGNAL(scaleDivChanged()), this, SLOT(yLeftScaleDivChanged()));
  connect(ui_->plot->axisWidget(QwtPlot::yRight), 
    SIGNAL(scaleDivChanged()), this, SLOT(yRightScaleDivChanged()));
  
  ui_->plot->axisWidget(QwtPlot::yLeft)->installEventFilter(this);
  ui_->plot->axisWidget(QwtPlot::yRight)->installEventFilter(this);
}

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
      ui_->plot->axisWidget(QwtPlot::yRight)->width(), 20);
  }
  
  return false;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotWidget::titleTextChanged(const QString& text) {
  QFontMetrics fontMetrics(ui_->lineEditTitle->font());
  
  ui_->lineEditTitle->setMinimumWidth(
    std::max(100, fontMetrics.width(text)+10));
}

void PlotWidget::runPauseClicked() {
}

void PlotWidget::clearClicked() {
  clear();
}

void PlotWidget::ejectClicked() {
  PlotConfigDialog dialog(this);
  
  dialog.setWindowTitle(getTitle().isEmpty() ? "Configure Plot" :
    "Configure \""+getTitle()+"\"");
  dialog.getWidget()->setTitle(getTitle());
  
  dialog.exec();
}

void PlotWidget::xTopScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::xBottom, *ui_->plot->axisScaleDiv(
    QwtPlot::xTop));
}

void PlotWidget::xBottomScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::xTop, *ui_->plot->axisScaleDiv(
    QwtPlot::xBottom));
}

void PlotWidget::yLeftScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::yRight, *ui_->plot->axisScaleDiv(
    QwtPlot::yLeft));
}

void PlotWidget::yRightScaleDivChanged() {
  ui_->plot->setAxisScaleDiv(QwtPlot::yLeft, *ui_->plot->axisScaleDiv(
    QwtPlot::yRight));
}

}
