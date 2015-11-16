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
  QComboBox(parent),
  registry_(new MessageTypeRegistry(this)),
  isUpdating_(false) {    
  connect(registry_, SIGNAL(updateStarted()), this,
    SLOT(registryUpdateStarted()));
  connect(registry_, SIGNAL(updateFinished()), this,
    SLOT(registryUpdateFinished()));
  
  connect(this, SIGNAL(currentIndexChanged(const QString&)), this,
    SLOT(currentIndexChanged(const QString&)));
}

MessageTypeComboBox::~MessageTypeComboBox() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageTypeComboBox::setEditable(bool editable) {
  if (editable != QComboBox::isEditable()) {
    QComboBox::setEditable(editable);
    
    if (lineEdit())
      connect(lineEdit(), SIGNAL(editingFinished()), this,
        SLOT(lineEditEditingFinished()));
  }
}

void MessageTypeComboBox::setCurrentType(const QString& type) {
  int index = findText(type);
  
  if (index < 0) {
    setEditText(type);
    lineEditEditingFinished();
  }
  else
    setCurrentIndex(index);
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
  setEditText("Updating...");
}

void MessageTypeComboBox::registryUpdateFinished() {
  QList<QString> types = registry_->getTypes();
  
  blockSignals(true);
  for (QList<QString>::const_iterator it = types.begin();
      it != types.end(); ++it)
    addItem(*it);
  blockSignals(false);
  
  if (!currentType_.isEmpty())
    setCurrentType(currentType_);
  else if (count())
    currentIndexChanged(currentText());
  
  isUpdating_= false;
  emit updateFinished();
  
  setEnabled(true);
}

void MessageTypeComboBox::currentIndexChanged(const QString& text) {
  if (currentType_ != text) {
    currentType_ = text;
    
    emit currentTypeChanged(currentType_);
  }
}

void MessageTypeComboBox::lineEditEditingFinished() {
  if (currentType_ != currentText()) {
    currentType_ = currentText();
    
    emit currentTypeChanged(currentType_);
  }
}

}
