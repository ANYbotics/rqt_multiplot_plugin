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

#include "rqt_multiplot/UrlComboBox.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

UrlComboBox::UrlComboBox(QWidget* parent) :
  QComboBox(parent),
  completer_(new UrlCompleter(this)) {
  connect(this, SIGNAL(activated(int)), this, SLOT(activated(int)));
  connect(this, SIGNAL(currentIndexChanged(const QString&)), this,
    SLOT(currentIndexChanged(const QString&)));
}

UrlComboBox::~UrlComboBox() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void UrlComboBox::setEditable(bool editable) {
  if (editable != QComboBox::isEditable()) {
    QComboBox::setEditable(editable);
    
    if (lineEdit()) {
      lineEdit()->setCompleter(completer_);

      connect(lineEdit(), SIGNAL(editingFinished()), this,
        SLOT(lineEditEditingFinished()));
    }
  }
}

UrlCompleter* UrlComboBox::getCompleter() const {
  return completer_;
}

void UrlComboBox::setCurrentUrl(const QString& url) {
  int index = findText(url);
  
  if (index < 0) {
    setEditText(url);
    lineEditEditingFinished();
  }
  else
    setCurrentIndex(index);
}

QString UrlComboBox::getCurrentUrl() const {
  return currentUrl_;
}

bool UrlComboBox::isCurrentUrlSelectable() const {
  return (findText(currentUrl_) >= 0);
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void UrlComboBox::activated(int index) {
  if (currentUrl_ != itemText(index)) {
    currentUrl_ = itemText(index);
    
    emit currentUrlChanged(currentUrl_);
  }
}

void UrlComboBox::currentIndexChanged(const QString& text) {
  if (currentUrl_ != text) {
    currentUrl_ = text;
    
    emit currentUrlChanged(currentUrl_);
  }
}

void UrlComboBox::lineEditEditingFinished() {
  if (currentUrl_ != currentText()) {
    currentUrl_ = currentText();
    
    emit currentUrlChanged(currentUrl_);
  }
}

}
