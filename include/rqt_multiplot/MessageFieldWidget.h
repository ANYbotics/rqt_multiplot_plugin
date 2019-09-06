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

#ifndef RQT_MULTIPLOT_MESSAGE_FIELD_WIDGET_H
#define RQT_MULTIPLOT_MESSAGE_FIELD_WIDGET_H

#include <QGridLayout>
#include <QTimer>
#include <QWidget>

#include <rqt_multiplot/MessageDefinitionLoader.h>
#include <rqt_multiplot/MessageFieldLineEdit.h>
#include <rqt_multiplot/MessageFieldTreeWidget.h>
#include <rqt_multiplot/MessageSubscriberRegistry.h>

namespace Ui {
  class MessageFieldWidget;
};

namespace rqt_multiplot {
  class MessageFieldWidget :
    public QWidget {
  Q_OBJECT
  public:
    MessageFieldWidget(QWidget* parent = 0);
    virtual ~MessageFieldWidget();
  
    QString getCurrentMessageType() const;
    variant_topic_tools::MessageDataType getCurrentMessageDataType() const;
    void setCurrentField(const QString& field);
    QString getCurrentField() const;
    variant_topic_tools::DataType getCurrentFieldDataType() const;
    bool isLoading() const;
    bool isConnecting() const;
    bool isCurrentFieldDefined() const;
    
    void loadFields(const QString& type);
    void connectTopic(const QString& topic, double timeout = 0.0);
    
  signals:
    void loadingStarted();
    void loadingFinished();
    void loadingFailed(const QString& error);
    
    void connecting(const QString& topic);
    void connected(const QString& topic);
    void connectionTimeout(const QString& topic, double timeout);
    
    void currentFieldChanged(const QString& field);
    
  private:
    Ui::MessageFieldWidget* ui_;
    
    QString currentField_;
    
    MessageDefinitionLoader* loader_;
    bool isLoading_;
    
    MessageSubscriberRegistry* registry_;
    bool isConnecting_;
    QString subscribedTopic_;
    QTimer* connectionTimer_;
    
    void disconnect();
    
  private slots:
    void loaderLoadingStarted();
    void loaderLoadingFinished();
    void loaderLoadingFailed(const QString& error);
    
    void subscriberMessageReceived(const QString& topic, const Message&
      message);
    
    void connectionTimerTimeout();
    
    void lineEditCurrentFieldChanged(const QString& field);
    void treeWidgetCurrentFieldChanged(const QString& field);
  };
};

#endif
