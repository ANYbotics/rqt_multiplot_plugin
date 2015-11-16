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

#include <rqt_multiplot/CurveConfigDialog.h>
#include <rqt_multiplot/CurveConfigWidget.h>

#include <ui_PlotConfigWidget.h>

#include "rqt_multiplot/PlotConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotConfigWidget::PlotConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::PlotConfigWidget()) {
  ui_->setupUi(this);  
  
  ui_->pushButtonAdd->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/add.png"))));
  ui_->pushButtonEdit->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/edit.png"))));
  ui_->pushButtonRemove->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/remove.png"))));
  
  connect(ui_->pushButtonAdd, SIGNAL(clicked()), this, SLOT(addClicked()));
  connect(ui_->pushButtonEdit, SIGNAL(clicked()), this, SLOT(editClicked()));
  connect(ui_->pushButtonRemove, SIGNAL(clicked()), this,
    SLOT(removeClicked()));
}

PlotConfigWidget::~PlotConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PlotConfigWidget::setTitle(const QString& title) {
  ui_->lineEditTitle->setText(title);
}

QString PlotConfigWidget::getTitle() const {
  return ui_->lineEditTitle->text();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void PlotConfigWidget::addClicked() {
  CurveConfigDialog dialog(this);
  
  dialog.setWindowTitle(getTitle().isEmpty() ? "Add Curve to Plot" :
    "Add Curve to \""+getTitle()+"\"");
  dialog.getWidget()->setTitle("Untitled Curve");
  
  dialog.exec();
}

void PlotConfigWidget::editClicked() {
}

void PlotConfigWidget::removeClicked() {
}

}
