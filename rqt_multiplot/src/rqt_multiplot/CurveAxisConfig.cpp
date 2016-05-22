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

#include "rqt_multiplot/CurveAxisConfig.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CurveAxisConfig::CurveAxisConfig(QObject* parent, const QString& topic,
    const QString& type, FieldType fieldType, const QString& field) :
  Config(parent),
  topic_(topic),
  type_(type),
  fieldType_(fieldType),
  field_(field),
  scaleConfig_(new CurveAxisScaleConfig(this)) {
  connect(scaleConfig_, SIGNAL(changed()), this, SLOT(scaleChanged()));
}

CurveAxisConfig::~CurveAxisConfig() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void CurveAxisConfig::setTopic(const QString& topic) {
  if (topic != topic_) {
    topic_ = topic;
    
    emit topicChanged(topic);
    emit changed();
  }
}

const QString& CurveAxisConfig::getTopic() const {
  return topic_;
}

void CurveAxisConfig::setType(const QString& type) {
  if (type != type_) {
    type_ = type;
    
    emit typeChanged(type);
    emit changed();
  }
}

const QString& CurveAxisConfig::getType() const {
  return type_;
}

void CurveAxisConfig::setFieldType(FieldType fieldType) {
  if (fieldType != fieldType_) {
    fieldType_ = fieldType;
    
    emit fieldTypeChanged(fieldType);
    emit changed();
  }
}

CurveAxisConfig::FieldType CurveAxisConfig::getFieldType() const {
  return fieldType_;
}

void CurveAxisConfig::setField(const QString& field) {
  if (field != field_) {
    field_ = field;
    
    emit fieldChanged(field);
    emit changed();
  }
}

const QString& CurveAxisConfig::getField() const {
  return field_;
}

CurveAxisScaleConfig* CurveAxisConfig::getScaleConfig() const {
  return scaleConfig_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CurveAxisConfig::save(QSettings& settings) const {
  settings.setValue("topic", topic_);
  settings.setValue("type", type_);
  settings.setValue("field_type", fieldType_);
  settings.setValue("field", field_);
  
  settings.beginGroup("scale");
  scaleConfig_->save(settings);
  settings.endGroup();
}

void CurveAxisConfig::load(QSettings& settings) {
  setTopic(settings.value("topic").toString());
  setType(settings.value("type").toString());
  setFieldType(static_cast<FieldType>(settings.value("field_type").toInt()));
  setField(settings.value("field").toString());
  
  settings.beginGroup("scale");
  scaleConfig_->load(settings);
  settings.endGroup();
}

void CurveAxisConfig::reset() {
  setTopic(QString());
  setType(QString());
  setFieldType(MessageData);
  setField(QString());
  
  scaleConfig_->reset();
}

void CurveAxisConfig::write(QDataStream& stream) const {
  stream << topic_;
  stream << type_;
  stream << (int)fieldType_;
  stream << field_;
  
  scaleConfig_->write(stream);
}

void CurveAxisConfig::read(QDataStream& stream) {
  QString topic, type, field;
  int fieldType;
  
  stream >> topic;
  setTopic(topic);
  stream >> type;
  setType(type);
  stream >> fieldType;
  setFieldType(static_cast<FieldType>(fieldType));
  stream >> field;
  setField(field);  
  
  scaleConfig_->read(stream);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

CurveAxisConfig& CurveAxisConfig::operator=(const CurveAxisConfig& src) {
  setTopic(src.topic_);
  setType(src.type_);
  setFieldType(src.fieldType_);
  setField(src.field_);
  
  *scaleConfig_ = *src.scaleConfig_;
  
  return *this;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void CurveAxisConfig::scaleChanged() {
  emit changed();
}

}
