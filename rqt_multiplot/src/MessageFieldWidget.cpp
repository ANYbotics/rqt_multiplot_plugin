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

#include <ui_MessageFieldWidget.h>

#include "rqt_multiplot/MessageFieldWidget.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldWidget::MessageFieldWidget(QWidget* parent) :
  QWidget(parent),
  ui_(new Ui::MessageFieldWidget()),
  loader_(new MessageDefinitionLoader(this)),
  isLoading_(false),
  registry_(new MessageSubscriberRegistry(this)),
  isConnecting_(false),
  connectionTimer_(new QTimer(this)) {
  ui_->setupUi(this);
  
  connectionTimer_->setSingleShot(true);    
    
  connect(loader_, SIGNAL(loadingStarted()), this,
    SLOT(loaderLoadingStarted()));
  connect(loader_, SIGNAL(loadingFinished()), this,
    SLOT(loaderLoadingFinished()));
  connect(loader_, SIGNAL(loadingFailed(const QString&)), this,
    SLOT(loaderLoadingFailed(const QString&)));
  
  connect(connectionTimer_, SIGNAL(timeout()), this,
    SLOT(connectionTimerTimeout()));
  
  connect(ui_->lineEdit, SIGNAL(currentFieldChanged(const QString&)),
    this, SLOT(lineEditCurrentFieldChanged(const QString&)));
  connect(ui_->treeWidget, SIGNAL(currentFieldChanged(const QString&)),
    this, SLOT(treeWidgetCurrentFieldChanged(const QString&)));
}

MessageFieldWidget::~MessageFieldWidget() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

QString MessageFieldWidget::getCurrentMessageType() const {
  return loader_->getType();
}

variant_topic_tools::MessageDataType MessageFieldWidget::
    getCurrentMessageDataType() const {
  return loader_->getDefinition().getMessageDataType();
}

void MessageFieldWidget::setCurrentField(const QString& field) {
  if (field != currentField_) {
    currentField_ = field;
    
    ui_->lineEdit->setCurrentField(field);
    ui_->treeWidget->setCurrentField(field);
    
    emit currentFieldChanged(field);
  }
}

QString MessageFieldWidget::getCurrentField() const {
  return currentField_;
}

variant_topic_tools::DataType MessageFieldWidget::
    getCurrentFieldDataType() const {
  return ui_->treeWidget->getCurrentFieldDataType();
}

bool MessageFieldWidget::isLoading() const {
  return isLoading_;
}

bool MessageFieldWidget::isConnecting() const {
  return isConnecting_;
}

bool MessageFieldWidget::isCurrentFieldDefined() const {
  return getCurrentFieldDataType().isValid();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageFieldWidget::loadFields(const QString& type) {
  if (isConnecting_) {
    disconnect();
    
    ui_->treeWidget->clear();
    setEnabled(true);
  }
  
  loader_->load(type);
}

void MessageFieldWidget::connectTopic(const QString& topic, double
    timeout) {
  loader_->wait();
  
  if (topic != subscribedTopic_) {
    if (isConnecting_) {
      disconnect();
      
      ui_->treeWidget->clear();
      setEnabled(true);
    }
  
    if (registry_->subscribe(topic, this, SLOT(subscriberMessageReceived(
        const QString&, const Message&)))) {
      setEnabled(false);
      
      isConnecting_ = true;
      subscribedTopic_ = topic;
      if (timeout > 0.0)
        connectionTimer_->start(timeout*1e3);
      emit connecting(topic);
      
      ui_->treeWidget->clear();
    }
  }
}

void MessageFieldWidget::disconnect() {
  registry_->unsubscribe(subscribedTopic_, this);
  
  isConnecting_ = false;
  subscribedTopic_.clear();
  connectionTimer_->stop();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageFieldWidget::loaderLoadingStarted() {
  setEnabled(false);  
  ui_->treeWidget->clear();
  
  isLoading_ = true;
  
  emit loadingStarted();
}

void MessageFieldWidget::loaderLoadingFinished() {
  ui_->lineEdit->setMessageDataType(loader_->getDefinition().
    getMessageDataType());
  ui_->treeWidget->setMessageDataType(loader_->getDefinition().
    getMessageDataType());
  
  ui_->lineEdit->setCurrentField(currentField_);
  ui_->treeWidget->setCurrentField(currentField_);
  
  setEnabled(true);
  
  isLoading_ = false;
  
  emit loadingFinished();  
}

void MessageFieldWidget::loaderLoadingFailed(const QString& error) {
  ui_->treeWidget->clear();
  
  isLoading_ = false;
  emit loadingFailed(error);
}

void MessageFieldWidget::subscriberMessageReceived(const QString& topic,
    const Message& message) {
  if (!isConnecting_)
    return;
  
  disconnect();
  
  ui_->lineEdit->setMessageDataType(message.getVariant().getType());
  ui_->treeWidget->setMessageDataType(message.getVariant().getType());
  
  ui_->lineEdit->setCurrentField(currentField_);
  ui_->treeWidget->setCurrentField(currentField_);
  
  setEnabled(true);
  
  emit connected(topic);
}

void MessageFieldWidget::connectionTimerTimeout() {
  if (!isConnecting_)
    return;
  
  QString topic = subscribedTopic_;
  double timeout = connectionTimer_->interval()*1e-3;
  
  disconnect();
  
  ui_->treeWidget->clear();
  
  emit connectionTimeout(topic, timeout);
}

void MessageFieldWidget::lineEditCurrentFieldChanged(const QString& field) {
  setCurrentField(field);
}

void MessageFieldWidget::treeWidgetCurrentFieldChanged(const QString& field) {
  setCurrentField(field);
}

}
