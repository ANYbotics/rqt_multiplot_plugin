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

#include <QCompleter>

#include <variant_topic_tools/MessageVariable.h>

#include "rqt_multiplot/MessageFieldLineEdit.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldLineEdit::MessageFieldLineEdit(QWidget* parent) :
  QLineEdit(parent),
  completer_(new MessageFieldCompleter(this)),
  completerModel_(new MessageFieldItemModel(this)) {
  completer_->setModel(completerModel_);
  setCompleter(completer_);
    
  connect(this, SIGNAL(editingFinished()), this, SLOT(editingFinished()));
}

MessageFieldLineEdit::~MessageFieldLineEdit() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageFieldLineEdit::setMessageDataType(const variant_topic_tools::
    MessageDataType& dataType) {
  completerModel_->setMessageDataType(dataType);
}

variant_topic_tools::MessageDataType MessageFieldLineEdit::
    getMessageDataType() const {
  return completerModel_->getMessageDataType();
}

void MessageFieldLineEdit::setCurrentField(const QString& field) {
  if (field != currentField_) {
    currentField_ = field;
    
    setText(field);
    
    emit currentFieldChanged(currentField_);
  }
}

QString MessageFieldLineEdit::getCurrentField() const {
  return currentField_;
}

variant_topic_tools::DataType MessageFieldLineEdit::
    getCurrentFieldDataType() const {
  return completerModel_->getFieldDataType(currentField_);
}

bool MessageFieldLineEdit::isCurrentFieldDefined() const {
  return getCurrentFieldDataType().isValid();
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageFieldLineEdit::editingFinished() {
  setCurrentField(text());
}

}
