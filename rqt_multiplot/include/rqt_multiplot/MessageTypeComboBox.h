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

#ifndef RQT_MULTIPLOT_MESSAGE_TYPE_COMBO_BOX_H
#define RQT_MULTIPLOT_MESSAGE_TYPE_COMBO_BOX_H

#include <rqt_multiplot/MatchFilterComboBox.h>
#include <rqt_multiplot/MessageTypeRegistry.h>

namespace rqt_multiplot {
  class MessageTypeComboBox :
    public MatchFilterComboBox {
  Q_OBJECT
  public:
    MessageTypeComboBox(QWidget* parent = 0);
    virtual ~MessageTypeComboBox();
    
    void setEditable(bool editable);
    void setCurrentType(const QString& type);
    QString getCurrentType() const;
    bool isUpdating() const;
    bool isCurrentTypeRegistered() const;
  
    void updateTypes();
    
  signals:
    void updateStarted();
    void updateFinished();
    void currentTypeChanged(const QString& type);
    
  private:
    QString currentType_;
    
    MessageTypeRegistry* registry_;
    bool isUpdating_;
    
  private slots:
    void registryUpdateStarted();
    void registryUpdateFinished();
    
    void currentIndexChanged(const QString& text);
    void lineEditEditingFinished();
  };
};

#endif
