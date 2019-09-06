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

#ifndef RQT_MULTIPLOT_MESSAGE_FIELD_LINE_EDIT_H
#define RQT_MULTIPLOT_MESSAGE_FIELD_LINE_EDIT_H

#include <QLineEdit>

#include <rqt_multiplot/MessageFieldCompleter.h>
#include <rqt_multiplot/MessageFieldItemModel.h>

#include <variant_topic_tools/MessageDataType.h>

namespace rqt_multiplot {
  class MessageFieldLineEdit :
    public QLineEdit {
  Q_OBJECT
  public:
    MessageFieldLineEdit(QWidget* parent = 0);
    virtual ~MessageFieldLineEdit();
  
    void setMessageDataType(const variant_topic_tools::MessageDataType&
      dataType);
    variant_topic_tools::MessageDataType getMessageDataType() const;
    void setCurrentField(const QString& field);
    QString getCurrentField() const;
    variant_topic_tools::DataType getCurrentFieldDataType() const;
    bool isCurrentFieldDefined() const;
    
  signals:
    void currentFieldChanged(const QString& field);
    
  private:
    QString currentField_;
    
    MessageFieldCompleter* completer_;
    MessageFieldItemModel* completerModel_;
    
  private slots:
    void editingFinished();
  };
};

#endif
