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

#include "rqt_multiplot/MessageFieldWidget.h"
#include <boost/concept_check.hpp>

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldWidget::MessageFieldWidget(QWidget* parent) :
  QWidget(parent),
  layout_(new QGridLayout(this)),
  lineEdit_(new MessageFieldLineEdit(this)),
  treeWidget_(new MessageFieldTreeWidget(this)),
  loader_(new MessageDefinitionLoader(this)),
  isLoading_(false),
  registry_(new MessageSubscriberRegistry(this)),
  isConnecting_(false),
  connectionTimer_(new QTimer(this)) {
  setLayout(layout_);
  
  layout_->setContentsMargins(0, 0, 0, 0);
  layout_->addWidget(lineEdit_, 0, 0);
  layout_->addWidget(treeWidget_, 1, 0);
    
  connectionTimer_->setSingleShot(true);    
    
  connect(loader_, SIGNAL(loadingStarted()), this,
    SLOT(loaderLoadingStarted()));
  connect(loader_, SIGNAL(loadingFinished()), this,
    SLOT(loaderLoadingFinished()));
  connect(loader_, SIGNAL(loadingFailed(const QString&)), this,
    SLOT(loaderLoadingFailed(const QString&)));
  
  connect(connectionTimer_, SIGNAL(timeout()), this,
    SLOT(connectionTimerTimeout()));
  
  connect(lineEdit_, SIGNAL(currentFieldChanged(const QString&)),
    this, SLOT(lineEditCurrentFieldChanged(const QString&)));
  connect(treeWidget_, SIGNAL(currentFieldChanged(const QString&)),
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
    lineEdit_->setCurrentField(field);
    treeWidget_->setCurrentField(field);
  }
}

QString MessageFieldWidget::getCurrentField() const {
  return currentField_;
}

variant_topic_tools::DataType MessageFieldWidget::
    getCurrentFieldDataType() const {
  return treeWidget_->getCurrentFieldDataType();
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
    
    treeWidget_->clear();
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
      
      treeWidget_->clear();
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
      
      treeWidget_->clear();
      
      QTreeWidgetItem* item = new QTreeWidgetItem();
      item->setText(0, "Connecting...");
      treeWidget_->addTopLevelItem(item);
    }
  }
}

void MessageFieldWidget::disconnect() {
  registry_->unsubscribe(subscribedTopic_, this,
    SLOT(subscriberMessageReceived(const QString&, const Message&)));
  
  isConnecting_ = false;
  subscribedTopic_.clear();
  connectionTimer_->stop();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageFieldWidget::loaderLoadingStarted() {
  setEnabled(false);
  
  isLoading_ = true;
  emit loadingStarted();
  
  treeWidget_->clear();
  
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, "Loading...");
  treeWidget_->addTopLevelItem(item);
}

void MessageFieldWidget::loaderLoadingFinished() {
  lineEdit_->setMessageDataType(loader_->getDefinition().
    getMessageDataType());
  treeWidget_->setMessageDataType(loader_->getDefinition().
    getMessageDataType());
  
  if (!currentField_.isEmpty())
    setCurrentField(currentField_);
  
  isLoading_ = false;
  emit loadingFinished();
  
  setEnabled(true);
}

void MessageFieldWidget::loaderLoadingFailed(const QString& error) {
  treeWidget_->clear();
  
  isLoading_ = false;
  emit loadingFailed(error);
}

void MessageFieldWidget::subscriberMessageReceived(const QString& topic,
    const Message& message) {
  if (!isConnecting_)
    return;
  
  disconnect();
  
  lineEdit_->setMessageDataType(message.getVariant().getType());
  treeWidget_->setMessageDataType(message.getVariant().getType());
  
  if (!currentField_.isEmpty())
    setCurrentField(currentField_);
  
  emit connected(topic);
  
  setEnabled(true);
}

void MessageFieldWidget::connectionTimerTimeout() {
  if (!isConnecting_)
    return;
  
  QString topic = subscribedTopic_;
  double timeout = connectionTimer_->interval()*1e-3;
  
  disconnect();
  
  treeWidget_->clear();
  
  emit connectionTimeout(topic, timeout);
}

void MessageFieldWidget::lineEditCurrentFieldChanged(const QString& field) {
  if (field != currentField_) {
    currentField_ = field;
    
    treeWidget_->blockSignals(true);
    treeWidget_->setCurrentField(field);
    treeWidget_->blockSignals(false);
    
    emit currentFieldChanged(currentField_);
  }
}

void MessageFieldWidget::treeWidgetCurrentFieldChanged(const QString& field) {
  if (field != currentField_) {
    currentField_ = field;
    
    lineEdit_->blockSignals(true);
    lineEdit_->setCurrentField(field);
    lineEdit_->blockSignals(false);
    
    emit currentFieldChanged(currentField_);
  }
}

}
