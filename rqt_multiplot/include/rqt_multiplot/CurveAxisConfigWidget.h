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

#ifndef RQT_MULTIPLOT_CURVE_AXIS_CONFIG_WIDGET_H
#define RQT_MULTIPLOT_CURVE_AXIS_CONFIG_WIDGET_H

#include <QWidget>

namespace Ui {
  class CurveAxisConfigWidget;
}

namespace rqt_multiplot {
  class CurveAxisConfigWidget :
    public QWidget {
  Q_OBJECT
  public:
    CurveAxisConfigWidget(QWidget* parent = 0);
    virtual ~CurveAxisConfigWidget();

    void setTopic(const QString& topic);
    QString getTopic() const;
    void setType(const QString& type);
    QString getType() const;
    void setField(const QString& field);
    QString getField() const;
    
    void updateTopics();
    void updateTypes();
    void updateFields();
    
  private:
    Ui::CurveAxisConfigWidget* ui_;
    
    bool validateTopic();
    bool validateType();
    bool validateField();
    
  private slots:
    void comboBoxTopicUpdateStarted();
    void comboBoxTopicUpdateFinished();
    void comboBoxTopicCurrentTopicChanged(const QString& topic);
    
    void comboBoxTypeUpdateStarted();
    void comboBoxTypeUpdateFinished();    
    void comboBoxTypeCurrentTypeChanged(const QString& type);
    
    void widgetFieldLoadingStarted();
    void widgetFieldLoadingFinished();
    void widgetFieldLoadingFailed(const QString& error);
    void widgetFieldConnecting(const QString& topic);
    void widgetFieldConnected(const QString& topic);
    void widgetFieldConnectionTimeout(const QString& topic, double timeout);
    void widgetFieldCurrentFieldChanged(const QString& field);
    
    void checkBoxFieldReceiptTimeStateChanged(int state);
  };
};

#endif
