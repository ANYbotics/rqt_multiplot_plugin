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

#include <ros/package.h>

#include <ui_ProgressWidget.h>

#include "rqt_multiplot/ProgressWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ProgressWidget::ProgressWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::ProgressWidget()),
  started_(false) {
  ui_->setupUi(this);
  
  ui_->progressBar->setMinimum(0);
  ui_->progressBar->setMaximum(100);
  ui_->progressBar->setValue(0);
  
  ui_->widgetStatus->setIcon(StatusWidget::Okay,
    QPixmap(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/okay.png"))));
  ui_->widgetStatus->setIcon(StatusWidget::Error,
    QPixmap(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/error.png"))));
  ui_->widgetStatus->setFrames(StatusWidget::Busy,
    QPixmap(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/busy.png"))), 8);
}

ProgressWidget::~ProgressWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void ProgressWidget::setCurrentProgress(double progress) {
  if (started_)
    ui_->progressBar->setValue(progress*1e2);
}

double ProgressWidget::getCurrentProgress() const {
  if (started_)
    return ui_->progressBar->value()*1e-2;
}

bool ProgressWidget::isStarted() const {
  return started_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ProgressWidget::start(const QString& toolTip) {
  if (!started_) {
    ui_->widgetStatus->setCurrentRole(StatusWidget::Busy, toolTip);
    
    ui_->progressBar->reset();
    ui_->progressBar->setTextVisible(true);
    
    started_ = true;
  }
}

void ProgressWidget::finish(const QString& toolTip) {
  if (started_) {
    ui_->widgetStatus->setCurrentRole(StatusWidget::Okay, toolTip);
    
    ui_->progressBar->reset();
    ui_->progressBar->setTextVisible(false);
    
    started_ = false;
  }
}

void ProgressWidget::fail(const QString& toolTip) {
  if (started_) {
    ui_->widgetStatus->setCurrentRole(StatusWidget::Error, toolTip);  
    
    ui_->progressBar->reset();
    ui_->progressBar->setTextVisible(false);
    
    started_ = false;
  }
}

}
