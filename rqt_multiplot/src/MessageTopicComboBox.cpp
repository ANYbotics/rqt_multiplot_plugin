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

#include <QAbstractItemView>
#include <QKeyEvent>
#include <QLineEdit>

#include "rqt_multiplot/MessageTopicComboBox.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageTopicComboBox::MessageTopicComboBox(QWidget* parent) :
  QComboBox(parent),
  completer_(new MatchFilterCompleter(this, Qt::MatchContains)),
  registry_(new MessageTopicRegistry(this)),
  isUpdating_(false) {
  connect(completer_, SIGNAL(activated(const QString&)), this,
    SLOT(completerActivated(const QString&)));
    
  connect(registry_, SIGNAL(updateStarted()), this,
    SLOT(registryUpdateStarted()));
  connect(registry_, SIGNAL(updateFinished()), this,
    SLOT(registryUpdateFinished()));
  
  connect(this, SIGNAL(currentIndexChanged(const QString&)), this,
    SLOT(currentIndexChanged(const QString&)));
  
  if (registry_->isUpdating())
    registryUpdateStarted();
  else if (!registry_->isEmpty())
    registryUpdateFinished();
  else
    registry_->update();
}

MessageTopicComboBox::~MessageTopicComboBox() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageTopicComboBox::setEditable(bool editable) {
  if (editable != QComboBox::isEditable()) {
    QComboBox::setEditable(editable);
    
    if (lineEdit()) {
      blockSignals(true);
  
      int index = findText(currentTopic_);
      
      if (index < 0)
        setEditText(currentTopic_);
      else
        setCurrentIndex(index);
      
      blockSignals(false);
      
      completer_->setModel(model());
      completer_->setWidget(this);
    
      connect(lineEdit(), SIGNAL(editingFinished()), this,
        SLOT(lineEditEditingFinished()));
    }
    else
      completer_->setModel(model());
  }
}

void MessageTopicComboBox::setCurrentTopic(const QString& topic) {
  if (topic != currentTopic_) {
    currentTopic_ = topic;
    
    int index = findText(topic);
    
    if (index < 0)
      setEditText(topic);
    else
      setCurrentIndex(index);
    
    emit currentTopicChanged(topic);
  }
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

void MessageTopicComboBox::keyPressEvent(QKeyEvent* event) {
  if (completer_->popup()->isVisible()) {
    switch (event->key()) {
      case Qt::Key_Escape:
      case Qt::Key_Tab:
      case Qt::Key_Backtab:
        event->ignore();
        return;
      case Qt::Key_Enter:
      case Qt::Key_Return:
        if (completer_->popup()->currentIndex().isValid()) {
          event->ignore();
          return; 
        }
        else
          completer_->popup()->hide();    
    }
  }

  bool isShortcut = (event->modifiers() & Qt::ControlModifier) &&
    (event->key() == Qt::Key_E);
  bool ctrlOrShift = event->modifiers() &
    (Qt::ControlModifier | Qt::ShiftModifier);
    
  if (!isShortcut)
    QComboBox::keyPressEvent(event);

  if (!isShortcut && !ctrlOrShift && (event->modifiers() != Qt::NoModifier)) {
    completer_->popup()->hide();    
    return;
  }

  if (count() >= 0) {
    completer_->setCompletionPrefix(currentText());
    completer_->complete();
    completer_->popup()->setCurrentIndex(QModelIndex());
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageTopicComboBox::completerActivated(const QString& text) {  
  if (lineEdit()) {
    setEditText(text);
    lineEdit()->selectAll();
  }
  
  setCurrentTopic(text);
}

void MessageTopicComboBox::registryUpdateStarted() {
  setEnabled(false);
  
  isUpdating_ = true;
  emit updateStarted();
  
  clear();
}

void MessageTopicComboBox::registryUpdateFinished() {
  QMap<QString, QString> topics = registry_->getTopics();
  
  blockSignals(true);
  
  for (QMap<QString, QString>::const_iterator it = topics.begin();
      it != topics.end(); ++it)
    addItem(it.key(), it.value());
  
  int index = findText(currentTopic_);
  
  if (index < 0)
    setEditText(currentTopic_);
  else
    setCurrentIndex(index);
  
  blockSignals(false);
  
  isUpdating_ = false;
  emit updateFinished();
  
  setEnabled(true);
}

void MessageTopicComboBox::currentIndexChanged(const QString& text) {
  if (currentIndex() >= 0)
    setCurrentTopic(text);
}

void MessageTopicComboBox::lineEditEditingFinished() {
  if (!completer_->popup()->isVisible())
    setCurrentTopic(currentText());
  else
    completer_->popup()->hide();
}

}
