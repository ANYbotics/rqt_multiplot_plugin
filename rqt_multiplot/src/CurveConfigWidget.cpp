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

#include <ui_CurveConfigWidget.h>

#include "rqt_multiplot/CurveConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveConfigWidget::CurveConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveConfigWidget()),
  config_(new CurveConfig(this)),
  messageTopicRegistry_(new MessageTopicRegistry(this)) {
  ui_->setupUi(this);

  ui_->pushButtonCopyRight->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/arrow_right.png"))));
  ui_->pushButtonCopyLeft->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/arrow_left.png"))));
  ui_->pushButtonSwap->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/22x22/arrows_right_left.png"))));
  
  ui_->curveAxisConfigWidgetX->setConfig(config_->getAxisConfig(
    CurveConfig::X));
  ui_->curveAxisConfigWidgetY->setConfig(config_->getAxisConfig(
    CurveConfig::Y));
  ui_->curveColorConfigWidget->setConfig(config_->getColorConfig());
  ui_->curveStyleConfigWidget->setConfig(config_->getStyleConfig());
  ui_->curveDataConfigWidget->setConfig(config_->getDataConfig());
  
  connect(config_, SIGNAL(titleChanged(const QString&)), this,
    SLOT(configTitleChanged(const QString&)));
  connect(config_, SIGNAL(subscriberQueueSizeChanged(size_t)), this,
    SLOT(configSubscriberQueueSizeChanged(size_t)));
  
  connect(config_->getAxisConfig(CurveConfig::X),
    SIGNAL(topicChanged(const QString&)), this,
    SLOT(configAxisConfigTopicChanged(const QString&)));
  connect(config_->getAxisConfig(CurveConfig::Y),
    SIGNAL(topicChanged(const QString&)), this,
    SLOT(configAxisConfigTopicChanged(const QString&)));

  connect(config_->getAxisConfig(CurveConfig::X),
    SIGNAL(typeChanged(const QString&)), this,
    SLOT(configAxisConfigTypeChanged(const QString&)));
  connect(config_->getAxisConfig(CurveConfig::Y),
    SIGNAL(typeChanged(const QString&)), this,
    SLOT(configAxisConfigTypeChanged(const QString&)));
  
  connect(ui_->lineEditTitle, SIGNAL(editingFinished()), this,
    SLOT(lineEditTitleEditingFinished()));
  connect(ui_->pushButtonCopyRight, SIGNAL(clicked()), this,
    SLOT(pushButtonCopyRightClicked()));
  connect(ui_->pushButtonCopyLeft, SIGNAL(clicked()), this,
    SLOT(pushButtonCopyLeftClicked()));
  connect(ui_->pushButtonSwap, SIGNAL(clicked()), this,
    SLOT(pushButtonSwapClicked()));
  connect(ui_->spinBoxSubscriberQueueSize, SIGNAL(valueChanged(int)),
    this, SLOT(spinBoxSubscriberQueueSizeValueChanged(int)));
  
  messageTopicRegistry_->update();
  
  configTitleChanged(config_->getTitle());
  configSubscriberQueueSizeChanged(config_->getSubscriberQueueSize());
}

CurveConfigWidget::~CurveConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveConfigWidget::setConfig(const CurveConfig& config) {
  *config_ = config;
}

CurveConfig& CurveConfigWidget::getConfig() {
  return *config_;
}

const CurveConfig& CurveConfigWidget::getConfig() const {
  return *config_;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveConfigWidget::configTitleChanged(const QString& title) {
  ui_->lineEditTitle->setText(title);
}

void CurveConfigWidget::configSubscriberQueueSizeChanged(size_t queueSize) {
  ui_->spinBoxSubscriberQueueSize->setValue(queueSize);
}

void CurveConfigWidget::configAxisConfigTopicChanged(const QString& topic) {
  CurveAxisConfig* source = static_cast<CurveAxisConfig*>(sender());
  CurveAxisConfig* destination = 0;
  
  if (source == config_->getAxisConfig(CurveConfig::X))
    destination = config_->getAxisConfig(CurveConfig::Y);
  else
    destination = config_->getAxisConfig(CurveConfig::X);  
  
  if (destination->getTopic().isEmpty())
    destination->setTopic(topic);
}

void CurveConfigWidget::configAxisConfigTypeChanged(const QString& type) {
  CurveAxisConfig* source = static_cast<CurveAxisConfig*>(sender());
  CurveAxisConfig* destination = 0;
  
  if (source == config_->getAxisConfig(CurveConfig::X))
    destination = config_->getAxisConfig(CurveConfig::Y);
  else
    destination = config_->getAxisConfig(CurveConfig::X);  
  
  if (destination->getType().isEmpty())
    destination->setType(type);
}

void CurveConfigWidget::lineEditTitleEditingFinished() {
  config_->setTitle(ui_->lineEditTitle->text());
}

void CurveConfigWidget::pushButtonCopyRightClicked() {
  *config_->getAxisConfig(CurveConfig::Y) = *config_->getAxisConfig(
    CurveConfig::X);
}

void CurveConfigWidget::pushButtonCopyLeftClicked() {
  *config_->getAxisConfig(CurveConfig::X) = *config_->getAxisConfig(
    CurveConfig::Y);
}

void CurveConfigWidget::pushButtonSwapClicked() {
  CurveAxisConfig xAxisConfig;
  
  xAxisConfig = *config_->getAxisConfig(CurveConfig::X);
  *config_->getAxisConfig(CurveConfig::X) = *config_->getAxisConfig(
    CurveConfig::Y);
  *config_->getAxisConfig(CurveConfig::Y) = xAxisConfig;
}

void CurveConfigWidget::spinBoxSubscriberQueueSizeValueChanged(int value) {
  config_->setSubscriberQueueSize(value);
}

}
