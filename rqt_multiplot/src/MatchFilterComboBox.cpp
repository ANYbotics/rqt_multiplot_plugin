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

#include "rqt_multiplot/MatchFilterComboBox.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MatchFilterComboBox::MatchFilterComboBox(QWidget* parent) :
  QComboBox(parent),
  matchFilterCompleter_(new MatchFilterCompleter(this, Qt::MatchContains)) {
  connect(matchFilterCompleter_, SIGNAL(activated(const QString&)), this,
    SLOT(matchFilterCompleterActivated(const QString&)));
}

MatchFilterComboBox::~MatchFilterComboBox() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MatchFilterComboBox::setEditable(bool editable) {
  if (editable != QComboBox::isEditable()) {
    QComboBox::setEditable(editable);
    
    if (lineEdit()) {
      matchFilterCompleter_->setModel(model());
      matchFilterCompleter_->setWidget(this);
      
      connect(lineEdit(), SIGNAL(editingFinished()), this,
        SLOT(lineEditEditingFinished()));
    }
    else
      matchFilterCompleter_->setModel(model());
  }
}

MatchFilterCompleter* MatchFilterComboBox::getMatchFilterCompleter() const {
  return matchFilterCompleter_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MatchFilterComboBox::keyPressEvent(QKeyEvent* event) {
  bool doComplete = (count() >= 0);
  
  if (matchFilterCompleter_->popup()->isVisible()) {
    switch (event->key()) {
      case Qt::Key_Escape:
      case Qt::Key_Tab:
      case Qt::Key_Backtab:
        event->ignore();
        return;
      case Qt::Key_Enter:
      case Qt::Key_Return:
        if (matchFilterCompleter_->popup()->currentIndex().isValid()) {
          event->ignore();
          return; 
        }
        else {
          matchFilterCompleter_->popup()->hide();    
          doComplete = false;
        }
    }
  }

  bool isShortcut = (event->modifiers() & Qt::ControlModifier) &&
    (event->key() == Qt::Key_E);
  bool ctrlOrShift = event->modifiers() &
    (Qt::ControlModifier | Qt::ShiftModifier);
    
  if (!isShortcut)
    QComboBox::keyPressEvent(event);

  if (!isShortcut && !ctrlOrShift && (event->modifiers() != Qt::NoModifier)) {
    matchFilterCompleter_->popup()->hide();    
    return;
  }

  if (doComplete) {
    matchFilterCompleter_->setCompletionPrefix(currentText());
    matchFilterCompleter_->complete();
    matchFilterCompleter_->popup()->setCurrentIndex(QModelIndex());
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MatchFilterComboBox::matchFilterCompleterActivated(const QString& text) {  
  setEditText(text);
  lineEdit()->selectAll();

  setCurrentIndex(findText(text));
  
  matchFilterCompleter_->popup()->hide();
}

void MatchFilterComboBox::lineEditEditingFinished() {
  if (!matchFilterCompleter_->popup()->isVisible()) {
    int index = findText(currentText());
    
    if (index < 0)
      setEditText(currentText());
    else
      setCurrentIndex(index);
  }
  else
    matchFilterCompleter_->popup()->hide();
}

}
