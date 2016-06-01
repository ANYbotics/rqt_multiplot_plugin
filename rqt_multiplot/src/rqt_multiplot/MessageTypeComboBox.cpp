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

#include "rqt_multiplot/MessageTypeComboBox.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageTypeComboBox::MessageTypeComboBox(QWidget* parent) :
  MatchFilterComboBox(parent),
  registry_(new MessageTypeRegistry(this)),
  isUpdating_(false) {    
  getMatchFilterCompleter()->setFilterCaseSensitivity(Qt::CaseInsensitive);
    
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

MessageTypeComboBox::~MessageTypeComboBox() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageTypeComboBox::setEditable(bool editable) {
  if (editable != QComboBox::isEditable()) {
    MatchFilterComboBox::setEditable(editable);
    
    if (lineEdit()) {
      blockSignals(true);
  
      int index = findText(currentType_);
      
      if (index < 0)
        setEditText(currentType_);
      else
        setCurrentIndex(index);
      
      blockSignals(false);
      
      connect(lineEdit(), SIGNAL(editingFinished()), this,
        SLOT(lineEditEditingFinished()));
    }
  }
}

void MessageTypeComboBox::setCurrentType(const QString& type) {
  if (type != currentType_) {
    currentType_ = type;
    
    int index = findText(type);
    
    if (index < 0)
      setEditText(type);
    else
      setCurrentIndex(index);
    
    emit currentTypeChanged(type);
  }
}

QString MessageTypeComboBox::getCurrentType() const {
  return currentType_;
}

bool MessageTypeComboBox::isUpdating() const {
  return isUpdating_;
}

bool MessageTypeComboBox::isCurrentTypeRegistered() const {
  return (findText(currentType_) >= 0);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageTypeComboBox::updateTypes() {
  registry_->update();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageTypeComboBox::registryUpdateStarted() {
  setEnabled(false);
  
  isUpdating_= true;
  emit updateStarted();
  
  clear();
}

void MessageTypeComboBox::registryUpdateFinished() {
  QList<QString> types = registry_->getTypes();
  
  blockSignals(true);
  
  for (QList<QString>::const_iterator it = types.begin();
      it != types.end(); ++it)
    addItem(*it);
  
  int index = findText(currentType_);
  
  if (index < 0)
    setEditText(currentType_);
  else
    setCurrentIndex(index);

  blockSignals(false);
  
  isUpdating_= false;
  emit updateFinished();
  
  setEnabled(true);
}

void MessageTypeComboBox::currentIndexChanged(const QString& text) {
  if (currentIndex() >= 0)
    setCurrentType(text);
}

void MessageTypeComboBox::lineEditEditingFinished() {
  setCurrentType(currentText());
}

}
