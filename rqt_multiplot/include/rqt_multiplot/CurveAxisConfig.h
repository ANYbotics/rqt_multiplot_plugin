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

#ifndef RQT_MULTIPLOT_CURVE_AXIS_CONFIG_H
#define RQT_MULTIPLOT_CURVE_AXIS_CONFIG_H

#include <QObject>
#include <QSettings>
#include <QString>

#include <rqt_multiplot/CurveAxisRange.h>

namespace rqt_multiplot {
  class CurveAxisConfig :
    public QObject {
  Q_OBJECT
  public:
    enum FieldType {
      MessageData,
      MessageReceiptTime
    };
    
    CurveAxisConfig(QObject* parent, const QString& topic = QString(),
      const QString& type = QString(), FieldType fieldType = MessageData,
      const QString& field = QString());
    ~CurveAxisConfig();

    void setTopic(const QString& topic);
    const QString& getTopic() const;
    void setType(const QString& type);
    const QString& getType() const;
    void setFieldType(FieldType fieldType);
    FieldType getFieldType() const;
    void setField(const QString& field);
    const QString& getField() const;
    CurveAxisRange* getRange() const;
  
    void save(QSettings& settings) const;
    void load(QSettings& settings);
    
    CurveAxisConfig& operator=(const CurveAxisConfig& src);
    
  signals:
    void topicChanged(const QString& topic);
    void typeChanged(const QString& type);
    void fieldTypeChanged(int fieldType);
    void fieldChanged(const QString& field);
    void changed();  
    
  private:
    QString topic_;
    QString type_;
    FieldType fieldType_;
    QString field_;
    
    CurveAxisRange* range_;
    
  private slots:
    void rangeChanged();
  };
};

#endif
