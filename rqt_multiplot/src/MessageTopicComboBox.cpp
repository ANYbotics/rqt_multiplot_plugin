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

#include <QLineEdit>

#include "rqt_multiplot/MessageTopicComboBox.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageTopicComboBox::MessageTopicComboBox(QWidget* parent) :
  QComboBox(parent),
  registry_(new MessageTopicRegistry(this)),
  isUpdating_(false) {
  connect(registry_, SIGNAL(updateStarted()), this,
    SLOT(registryUpdateStarted()));
  connect(registry_, SIGNAL(updateFinished()), this,
    SLOT(registryUpdateFinished()));
  
  connect(this, SIGNAL(currentIndexChanged(const QString&)), this,
    SLOT(currentIndexChanged(const QString&)));
}

MessageTopicComboBox::~MessageTopicComboBox() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageTopicComboBox::setEditable(bool editable) {
  if (editable != QComboBox::isEditable()) {
    QComboBox::setEditable(editable);
    
    if (lineEdit())
      connect(lineEdit(), SIGNAL(editingFinished()), this,
        SLOT(lineEditEditingFinished()));
  }
}

void MessageTopicComboBox::setCurrentTopic(const QString& topic) {
  int index = findText(topic);
  
  if (index < 0) {
    setEditText(topic);
    lineEditEditingFinished();
  }
  else
    setCurrentIndex(index);
}

QString MessageTopicComboBox::getCurrentTopic() const {
  return currentTopic_;
}

QString MessageTopicComboBox::getCurrentTopicType() const {
  int index = findText(currentTopic_);
  
  if (index >= 0)
    return itemData(index).toString();
  else
    return QString();
}

bool MessageTopicComboBox::isUpdating() const {
  return isUpdating_;
}

bool MessageTopicComboBox::isCurrentTopicRegistered() const {
  return (findText(currentTopic_) >= 0);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageTopicComboBox::updateTopics() {
  registry_->update();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageTopicComboBox::registryUpdateStarted() {
  setEnabled(false);
  
  isUpdating_ = true;
  emit updateStarted();
  
  clear();
  setEditText("Updating...");
}

void MessageTopicComboBox::registryUpdateFinished() {
  QMap<QString, QString> topics = registry_->getTopics();
  
  blockSignals(true);
  for (QMap<QString, QString>::const_iterator it = topics.begin();
      it != topics.end(); ++it)
    addItem(it.key(), it.value());
  blockSignals(false);
  
  if (!currentTopic_.isEmpty())
    setCurrentTopic(currentTopic_);
  else if (count())
    currentIndexChanged(currentText());
  
  isUpdating_ = false;
  emit updateFinished();
  
  setEnabled(true);
}

void MessageTopicComboBox::currentIndexChanged(const QString& text) {
  if (currentTopic_ != text) {
    currentTopic_ = text;
    
    emit currentTopicChanged(currentTopic_);
  }
}

void MessageTopicComboBox::lineEditEditingFinished() {
  if (currentTopic_ != currentText()) {
    currentTopic_ = currentText();
    
    emit currentTopicChanged(currentTopic_);
  }
}

}
