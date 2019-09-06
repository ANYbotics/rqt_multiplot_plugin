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

#include <ui_CurveAxisConfigWidget.h>

#include "rqt_multiplot/CurveAxisConfigWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveAxisConfigWidget::CurveAxisConfigWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::CurveAxisConfigWidget()),
  config_(0) {
  ui_->setupUi(this);

  QPixmap pixmapOkay = QPixmap(QString::fromStdString(ros::package::getPath(
    "rqt_multiplot").append("/resource/22x22/okay.png")));
  QPixmap pixmapError = QPixmap(QString::fromStdString(ros::package::getPath(
    "rqt_multiplot").append("/resource/22x22/error.png")));
  QPixmap pixmapBusy = QPixmap(QString::fromStdString(ros::package::getPath(
    "rqt_multiplot").append("/resource/22x22/busy.png")));
  
  ui_->statusWidgetTopic->setIcon(StatusWidget::Okay, pixmapOkay);
  ui_->statusWidgetTopic->setIcon(StatusWidget::Error, pixmapError);
  ui_->statusWidgetTopic->setFrames(StatusWidget::Busy, pixmapBusy, 8);
  
  ui_->statusWidgetType->setIcon(StatusWidget::Okay, pixmapOkay);
  ui_->statusWidgetType->setIcon(StatusWidget::Error, pixmapError);
  ui_->statusWidgetType->setFrames(StatusWidget::Busy, pixmapBusy, 8);

  ui_->statusWidgetField->setIcon(StatusWidget::Okay, pixmapOkay);
  ui_->statusWidgetField->setIcon(StatusWidget::Error, pixmapError);
  ui_->statusWidgetField->setFrames(StatusWidget::Busy, pixmapBusy, 8);
  
  ui_->statusWidgetScale->setIcon(StatusWidget::Okay, pixmapOkay);
  ui_->statusWidgetScale->setIcon(StatusWidget::Error, pixmapError);
  ui_->statusWidgetScale->setFrames(StatusWidget::Busy, pixmapBusy, 8);
  
  connect(ui_->comboBoxTopic, SIGNAL(updateStarted()), this,
    SLOT(comboBoxTopicUpdateStarted()));
  connect(ui_->comboBoxTopic, SIGNAL(updateFinished()), this,
    SLOT(comboBoxTopicUpdateFinished()));  
  connect(ui_->comboBoxTopic, SIGNAL(currentTopicChanged(const QString&)),
    this, SLOT(comboBoxTopicCurrentTopicChanged(const QString&)));
  
  connect(ui_->comboBoxType, SIGNAL(updateStarted()), this,
    SLOT(comboBoxTypeUpdateStarted()));
  connect(ui_->comboBoxType, SIGNAL(updateFinished()), this,
    SLOT(comboBoxTypeUpdateFinished()));  
  connect(ui_->comboBoxType, SIGNAL(currentTypeChanged(const QString&)),
    this, SLOT(comboBoxTypeCurrentTypeChanged(const QString&)));
  
  connect(ui_->widgetField, SIGNAL(loadingStarted()), this,
    SLOT(widgetFieldLoadingStarted()));
  connect(ui_->widgetField, SIGNAL(loadingFinished()), this,
    SLOT(widgetFieldLoadingFinished()));
  connect(ui_->widgetField, SIGNAL(loadingFailed(const QString&)),
    this, SLOT(widgetFieldLoadingFailed(const QString&)));
  connect(ui_->widgetField, SIGNAL(connecting(const QString&)), this,
    SLOT(widgetFieldConnecting(const QString&)));
  connect(ui_->widgetField, SIGNAL(connected(const QString&)), this,
    SLOT(widgetFieldConnected(const QString&)));
  connect(ui_->widgetField, SIGNAL(connectionTimeout(const QString&,
    double)), this, SLOT(widgetFieldConnectionTimeout(const QString&,
    double)));
  connect(ui_->widgetField, SIGNAL(currentFieldChanged(const QString&)),
    this, SLOT(widgetFieldCurrentFieldChanged(const QString&)));
  
  connect(ui_->checkBoxFieldReceiptTime, SIGNAL(stateChanged(int)), this,
    SLOT(checkBoxFieldReceiptTimeStateChanged(int)));
  
  if (ui_->comboBoxTopic->isUpdating())
    comboBoxTopicUpdateStarted();
  else
    comboBoxTopicUpdateFinished();
  
  if (ui_->comboBoxType->isUpdating())
    comboBoxTypeUpdateStarted();
  else
    comboBoxTypeUpdateFinished();
}

CurveAxisConfigWidget::~CurveAxisConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveAxisConfigWidget::setConfig(CurveAxisConfig* config) {
  if (config_ != config) {
    if (config_) {
      disconnect(config_, SIGNAL(topicChanged(const QString&)), this,
        SLOT(configTopicChanged(const QString&)));
      disconnect(config_, SIGNAL(typeChanged(const QString&)), this,
        SLOT(configTypeChanged(const QString&)));
      disconnect(config_, SIGNAL(fieldTypeChanged(int)), this,
        SLOT(configFieldTypeChanged(int)));
      disconnect(config_, SIGNAL(fieldChanged(const QString&)), this,
        SLOT(configFieldChanged(const QString&)));
      disconnect(config_->getScaleConfig(), SIGNAL(changed()), this,
        SLOT(configScaleConfigChanged()));
    }
    
    config_ = config;
    
    if (config) {
      ui_->widgetScale->setConfig(config->getScaleConfig());
            
      connect(config, SIGNAL(topicChanged(const QString&)), this,
        SLOT(configTopicChanged(const QString&)));
      connect(config, SIGNAL(typeChanged(const QString&)), this,
        SLOT(configTypeChanged(const QString&)));
      connect(config, SIGNAL(fieldTypeChanged(int)), this,
        SLOT(configFieldTypeChanged(int)));
      connect(config, SIGNAL(fieldChanged(const QString&)), this,
        SLOT(configFieldChanged(const QString&)));
      connect(config->getScaleConfig(), SIGNAL(changed()), this,
        SLOT(configScaleConfigChanged()));
      
      configTopicChanged(config->getTopic());
      configTypeChanged(config->getType());
      configFieldTypeChanged(config->getFieldType());
      configFieldChanged(config->getField());
      configScaleConfigChanged();
    }
    else {
      ui_->widgetScale->setConfig(0);
    }
  }
}

CurveAxisConfig* CurveAxisConfigWidget::getConfig() const {
  return config_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveAxisConfigWidget::updateTopics() {
  ui_->comboBoxTopic->updateTopics();
}

void CurveAxisConfigWidget::updateTypes() {
  ui_->comboBoxType->updateTypes();
}

void CurveAxisConfigWidget::updateFields() {
  if (config_)
    ui_->widgetField->loadFields(config_->getType());
}

bool CurveAxisConfigWidget::validateTopic() {
  if (!config_ || ui_->comboBoxTopic->isUpdating())
    return false;
  
  if (config_->getTopic().isEmpty()) {
    ui_->statusWidgetTopic->setCurrentRole(StatusWidget::Error,
      "No topic selected");
    
    return false;
  }
  
  if (ui_->comboBoxTopic->isCurrentTopicRegistered()) {
    ui_->statusWidgetTopic->setCurrentRole(StatusWidget::Okay,
      "Topic okay");
    
    return true;
  }
  else {
    ui_->statusWidgetTopic->setCurrentRole(StatusWidget::Error,
      "Topic ["+config_->getTopic()+"] not advertised");
    
    return false;
  }
}

bool CurveAxisConfigWidget::validateType() {
  if (!config_ || ui_->comboBoxType->isUpdating())
    return false;

  if (config_->getType().isEmpty()) {
    ui_->statusWidgetType->setCurrentRole(StatusWidget::Error,
      "No message type selected");
    
    return false;
  }
  
  if (!ui_->comboBoxTopic->isCurrentTopicRegistered()) {
    if (ui_->comboBoxType->isCurrentTypeRegistered()) {
      ui_->statusWidgetType->setCurrentRole(StatusWidget::Okay,
        "Message type okay");
      
      return true;
    }
    else {
      ui_->statusWidgetType->setCurrentRole(StatusWidget::Error,
        "Message type ["+config_->getType()+"] not found in package path");
      
      return false;
    }
  }
  else {
    if (ui_->comboBoxTopic->getCurrentTopicType() == config_->getType()) {
      ui_->statusWidgetType->setCurrentRole(StatusWidget::Okay,
        "Message type okay");
      
      return true;
    }
    else {
      ui_->statusWidgetType->setCurrentRole(StatusWidget::Error,
        "Message type ["+config_->getType()+
        "] mismatches advertised message type ["
        +ui_->comboBoxTopic->getCurrentTopicType()+"] for topic ["+
        config_->getTopic()+"]");
      
      return false;
    }
  }  
}

bool CurveAxisConfigWidget::validateField() {
  if (!config_ || ui_->widgetField->isLoading())
    return false;
  
  if (config_->getFieldType() == CurveAxisConfig::MessageReceiptTime) {
    ui_->statusWidgetField->setCurrentRole(StatusWidget::Okay,
      "Message field okay");
    
    return true;
  }
  
  if (config_->getField().isEmpty()) {
    ui_->statusWidgetField->setCurrentRole(StatusWidget::Error,
      "No message field selected");
    
    return false;
  }
  
  variant_topic_tools::DataType fieldType = ui_->widgetField->
    getCurrentFieldDataType();  
  
  if (fieldType.isValid()) {
    if (fieldType.isBuiltin() && variant_topic_tools::
        BuiltinDataType(fieldType).isNumeric()) {
      ui_->statusWidgetField->setCurrentRole(StatusWidget::Okay,
        "Message field okay");
    
      return true;
    }
    else {
      ui_->statusWidgetField->setCurrentRole(StatusWidget::Error,
        "Message field ["+config_->getField()+"] is not numeric");
      
      return false;
    }
  }
  else {
    ui_->statusWidgetField->setCurrentRole(StatusWidget::Error,
      "No such message field ["+config_->getField()+"]");
    
    return false;
  }
}

bool CurveAxisConfigWidget::validateScale() {
  if (!config_)
    return false;
  
  if (!config_->getScaleConfig()->isValid()) {
    ui_->statusWidgetScale->setCurrentRole(StatusWidget::Error,
      "Axis scale invalid");
    
    return false;
  }
  else {
    ui_->statusWidgetScale->setCurrentRole(StatusWidget::Okay,
      "Axis scale okay");
    
    return true;
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveAxisConfigWidget::configTopicChanged(const QString& topic) {
  ui_->comboBoxTopic->setCurrentTopic(topic);
  
  validateTopic();
}

void CurveAxisConfigWidget::configTypeChanged(const QString& type) {
  ui_->comboBoxType->setCurrentType(type);
  
  validateType();
}

void CurveAxisConfigWidget::configFieldTypeChanged(int fieldType) {
  ui_->checkBoxFieldReceiptTime->setCheckState(
    (fieldType == CurveAxisConfig::MessageReceiptTime) ?
    Qt::Checked : Qt::Unchecked);
  
  validateType();
}

void CurveAxisConfigWidget::configFieldChanged(const QString& field) {
  ui_->widgetField->setCurrentField(field);
  
  validateField();
}

void CurveAxisConfigWidget::configScaleConfigChanged() {
  validateScale();
}

void CurveAxisConfigWidget::comboBoxTopicUpdateStarted() {
  ui_->statusWidgetTopic->pushCurrentRole();
  ui_->statusWidgetTopic->setCurrentRole(StatusWidget::Busy,
    "Updating topics...");
}

void CurveAxisConfigWidget::comboBoxTopicUpdateFinished() {
  ui_->statusWidgetTopic->popCurrentRole();
  
  validateTopic();
}

void CurveAxisConfigWidget::comboBoxTopicCurrentTopicChanged(const QString&
    topic) {
  if (config_) {
    config_->setTopic(topic);
    
    if (ui_->comboBoxTopic->isCurrentTopicRegistered())
      config_->setType(ui_->comboBoxTopic->getCurrentTopicType());
  }
  
  validateTopic();
}

void CurveAxisConfigWidget::comboBoxTypeUpdateStarted() {
  ui_->statusWidgetType->pushCurrentRole();
  ui_->statusWidgetType->setCurrentRole(StatusWidget::Busy,
    "Updating message types...");
}

void CurveAxisConfigWidget::comboBoxTypeUpdateFinished() {
  ui_->statusWidgetType->popCurrentRole();
  
  validateType();
}

void CurveAxisConfigWidget::comboBoxTypeCurrentTypeChanged(const QString&
    type) {
  if (config_)
    config_->setType(type);
  
  validateType();
  updateFields();
}

void CurveAxisConfigWidget::widgetFieldLoadingStarted() {
  ui_->widgetField->setEnabled(false);
  
  ui_->statusWidgetField->pushCurrentRole();
  ui_->statusWidgetField->setCurrentRole(StatusWidget::Busy,
    "Loading message definition...");
}

void CurveAxisConfigWidget::widgetFieldLoadingFinished() {
  ui_->widgetField->setEnabled(ui_->checkBoxFieldReceiptTime->
    checkState() != Qt::Checked);
  ui_->statusWidgetField->popCurrentRole();
  
  validateField();
}

void CurveAxisConfigWidget::widgetFieldLoadingFailed(const QString&
    error) {
  ui_->statusWidgetField->popCurrentRole();
  
  if (config_ && (ui_->comboBoxTopic->getCurrentTopicType() == config_->
      getType()))
    ui_->widgetField->connectTopic(config_->getTopic());
  else
    validateField();
}

void CurveAxisConfigWidget::widgetFieldConnecting(const QString& topic) {
  ui_->widgetField->setEnabled(false);
  
  ui_->statusWidgetField->pushCurrentRole();
  ui_->statusWidgetField->setCurrentRole(StatusWidget::Busy,
    "Waiting for connnection on topic ["+topic+"]...");
}

void CurveAxisConfigWidget::widgetFieldConnected(const QString& topic) {
  ui_->widgetField->setEnabled(ui_->checkBoxFieldReceiptTime->
    checkState() != Qt::Checked);
  ui_->statusWidgetField->popCurrentRole();  
  
  validateField();
}

void CurveAxisConfigWidget::widgetFieldConnectionTimeout(const QString&
    topic, double timeout) {
  ui_->statusWidgetField->popCurrentRole();
  
  validateField();
}

void CurveAxisConfigWidget::widgetFieldCurrentFieldChanged(const QString&
    field) {
  if (config_)
    config_->setField(field);
  
  validateField();
}

void CurveAxisConfigWidget::checkBoxFieldReceiptTimeStateChanged(int state) {
  ui_->widgetField->setEnabled(state != Qt::Checked);
  
  if (config_)
    config_->setFieldType((state == Qt::Checked) ?
      CurveAxisConfig::MessageReceiptTime : CurveAxisConfig::MessageData);
    
  validateField();
}

}
