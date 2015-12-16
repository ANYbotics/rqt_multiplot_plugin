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

#ifndef RQT_MULTIPLOT_MESSAGE_TOPIC_COMBO_BOX_H
#define RQT_MULTIPLOT_MESSAGE_TOPIC_COMBO_BOX_H

#include <rqt_multiplot/MatchFilterComboBox.h>
#include <rqt_multiplot/MessageTopicRegistry.h>

namespace rqt_multiplot {
  class MessageTopicComboBox :
    public MatchFilterComboBox {
  Q_OBJECT
  public:
    MessageTopicComboBox(QWidget* parent = 0);
    virtual ~MessageTopicComboBox();
    
    void setEditable(bool editable);
    void setCurrentTopic(const QString& topic);
    QString getCurrentTopic() const;
    QString getCurrentTopicType() const;
    bool isUpdating() const;
    bool isCurrentTopicRegistered() const;
  
    void updateTopics();
    
  signals:
    void updateStarted();
    void updateFinished();
    void currentTopicChanged(const QString& topic);
    
  private:
    QString currentTopic_;
    
    MessageTopicRegistry* registry_;
    bool isUpdating_;
    
  private slots:
    void registryUpdateStarted();
    void registryUpdateFinished();
    
    void currentIndexChanged(const QString& text);
    void lineEditEditingFinished();
  };
};

#endif
