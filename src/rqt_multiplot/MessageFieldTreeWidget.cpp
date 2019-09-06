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

#include <limits>

#include <QHeaderView>
#include <QSpinBox>

#include <variant_topic_tools/MessageVariable.h>

#include "rqt_multiplot/MessageFieldTreeWidget.h"

Q_DECLARE_METATYPE(variant_topic_tools::DataType)

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldTreeWidget::MessageFieldTreeWidget(QWidget* parent) :
  QTreeWidget(parent) {
  setColumnCount(2);
  headerItem()->setText(0, "Name");
  headerItem()->setText(1, "Type");

#if QT_VERSION >= 0x050000
  header()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else
  header()->setResizeMode(QHeaderView::ResizeToContents);
#endif

  connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*,
    QTreeWidgetItem*)), this, SLOT(currentItemChanged(QTreeWidgetItem*,
    QTreeWidgetItem*)));
}

MessageFieldTreeWidget::~MessageFieldTreeWidget() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageFieldTreeWidget::setMessageDataType(const variant_topic_tools::
    MessageDataType& dataType) {
  clear();
  
  blockSignals(true);
  invisibleRootItem()->setData(1, Qt::UserRole, QVariant::
    fromValue<variant_topic_tools::DataType>(dataType));
  for (size_t i = 0; i < dataType.getNumVariableMembers(); ++i)
    addField(dataType.getVariableMember(i));
  blockSignals(false);
  
  if (!currentField_.isEmpty())
    setCurrentItem(currentField_);
}

variant_topic_tools::MessageDataType MessageFieldTreeWidget::
    getMessageDataType() const {
  QTreeWidgetItem* item = invisibleRootItem();
  
  if (item)
    return item->data(1, Qt::UserRole).value<variant_topic_tools::DataType>();
  else
    return variant_topic_tools::DataType();
}

void MessageFieldTreeWidget::setCurrentField(const QString& field) {
  if (field != currentField_) {
    currentField_ = field;
    
    setCurrentItem(field);
    
    emit currentFieldChanged(field);
  }
}

QString MessageFieldTreeWidget::getCurrentField() const {
  return currentField_;
}

variant_topic_tools::DataType MessageFieldTreeWidget::
    getCurrentFieldDataType() const {
  QTreeWidgetItem* item = currentItem();

  if (item)
    return item->data(1, Qt::UserRole).value<variant_topic_tools::DataType>();
  else
    return variant_topic_tools::DataType();
}

bool MessageFieldTreeWidget::isCurrentFieldDefined() const {
  return getCurrentFieldDataType().isValid();
}

void MessageFieldTreeWidget::setCurrentItem(const QString& field) {
  QTreeWidgetItem* item = invisibleRootItem();
  QStringList fields = field.split("/");

  while (item && !fields.isEmpty()) {
    QVariant itemData = item->data(1, Qt::UserRole);
    
    if (itemData.isValid()) {
      variant_topic_tools::DataType fieldType = itemData.
        value<variant_topic_tools::DataType>();
        
      if (fieldType.isMessage()) {
        QTreeWidgetItem* childItem = findChild(item, 0, fields.front());
        
        if (childItem) {
          item = childItem;
          fields.removeFirst();
          
          continue;
        }
      }
      else if (fieldType.isArray()) {
        bool indexOkay = false;
        size_t index = fields.front().toUInt(&indexOkay);
        QSpinBox* spinBoxIndex = static_cast<QSpinBox*>(
          itemWidget(item->child(0), 0));
        
        if (indexOkay && (index < spinBoxIndex->maximum())) {
          spinBoxIndex->blockSignals(true);
          spinBoxIndex->setValue(index);
          spinBoxIndex->blockSignals(false);
          
          item = item->child(0);
          fields.removeFirst();
          
          continue;
        }
      }
    }
    
    item = invisibleRootItem();
    break;
  }
  
  blockSignals(true);
  QTreeWidget::setCurrentItem(item);
  blockSignals(false);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageFieldTreeWidget::addField(const variant_topic_tools::
    MessageVariable& variable, QTreeWidgetItem* parent) {
  QTreeWidgetItem* item = new QTreeWidgetItem();
  
  item->setText(0, QString::fromStdString(variable.getName()));
  item->setText(1, QString::fromStdString(variable.getType().
    getIdentifier()));
  item->setData(1, Qt::UserRole, QVariant::fromValue(variable.getType()));
  item->setFlags(Qt::ItemIsEnabled);
  
  QFont typeFont = item->font(1);
  typeFont.setItalic(true);
  item->setFont(1, typeFont);
  
  if (parent)
    parent->addChild(item);
  else
    addTopLevelItem(item);
  
  if (variable.getType().isMessage()) {
    variant_topic_tools::MessageDataType messageType = variable.getType();
    
    for (size_t i = 0; i < messageType.getNumVariableMembers(); ++i)
      addField(messageType.getVariableMember(i), item);
  }
  else if (variable.getType().isArray()) {
    variant_topic_tools::ArrayDataType arrayType = variable.getType();
    
    QSpinBox* spinBoxIndex = new QSpinBox(this);
    spinBoxIndex->setMinimum(0);
    if (!arrayType.isDynamic())
      spinBoxIndex->setMaximum(arrayType.getNumMembers()-1);
    else
      spinBoxIndex->setMaximum(std::numeric_limits<int>::max());
    spinBoxIndex->setFrame(false);
    
    connect(spinBoxIndex, SIGNAL(valueChanged(int)), this,
      SLOT(spinBoxIndexValueChanged(int)));
    
    QTreeWidgetItem* memberItem = new QTreeWidgetItem();
    memberItem->setText(1, QString::fromStdString(arrayType.getMemberType().
      getIdentifier()));
    memberItem->setData(1, Qt::UserRole, QVariant::fromValue(arrayType.
      getMemberType()));
    memberItem->setFlags(Qt::ItemIsEnabled);
    
    QFont memberTypeFont = memberItem->font(1);
    memberTypeFont.setItalic(true);
    memberItem->setFont(1, memberTypeFont);
    
    item->addChild(memberItem);
    setItemWidget(memberItem, 0, spinBoxIndex);
    
    if (arrayType.getMemberType().isMessage()) {
      variant_topic_tools::MessageDataType messageMemberType = arrayType.
        getMemberType();
      
      for (size_t i = 0; i < messageMemberType.getNumVariableMembers(); ++i)
        addField(messageMemberType.getVariableMember(i), memberItem);
    }
    else if (arrayType.getMemberType().isBuiltin()) {
      variant_topic_tools::BuiltinDataType builtinMemberType = arrayType.
        getMemberType();
        
      if (builtinMemberType.isNumeric())
        memberItem->setFlags(memberItem->flags() | Qt::ItemIsSelectable);
    }
  }
  else if (variable.getType().isBuiltin()) {
    variant_topic_tools::BuiltinDataType builtinType = variable.getType();
    
    if (builtinType.isNumeric())
      item->setFlags(item->flags() | Qt::ItemIsSelectable);
  }
}

QTreeWidgetItem* MessageFieldTreeWidget::findChild(QTreeWidgetItem* item,
    int column, const QString& text) const {
  for (size_t i = 0; i < item->childCount(); ++i) {
    if (item->child(i)->text(column) == text)
      return item->child(i);
  }
  
  return 0;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MessageFieldTreeWidget::currentItemChanged(QTreeWidgetItem* current,
    QTreeWidgetItem* previous) {
  QString field;
  
  while (current) {
    QString text = current->text(0);
    
    if (text.isEmpty()) {
      QSpinBox* spinBoxIndex = static_cast<QSpinBox*>(
        itemWidget(current, 0));
      
      text = QString::number(spinBoxIndex->value());
    }
    
    if (!field.isEmpty())
      field = text+"/"+field;
    else
      field = text;
      
    current = current->parent();
  }

  setCurrentField(field);
}

void MessageFieldTreeWidget::spinBoxIndexValueChanged(int value) {
  currentItemChanged(currentItem(), currentItem());
}

}
