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

#ifndef RQT_MULTIPLOT_CURVE_CONFIG_H
#define RQT_MULTIPLOT_CURVE_CONFIG_H

#include <QMap>
#include <QString>

#include <rqt_multiplot/Config.h>
#include <rqt_multiplot/CurveAxisConfig.h>
#include <rqt_multiplot/CurveColorConfig.h>
#include <rqt_multiplot/CurveDataConfig.h>
#include <rqt_multiplot/CurveStyleConfig.h>

namespace rqt_multiplot {
  class CurveConfig :
    public Config {
  Q_OBJECT
  public:
    enum Axis {
      X,
      Y
    };

    static const QString MimeType;
    
    CurveConfig(QObject* parent = 0, const QString& title = "Untitled Curve",
      size_t subscriberQueueSize = 100);
    ~CurveConfig();

    void setTitle(const QString& title);
    const QString& getTitle() const;
    CurveAxisConfig* getAxisConfig(Axis axis) const;
    CurveColorConfig* getColorConfig() const;
    CurveStyleConfig* getStyleConfig() const;
    CurveDataConfig* getDataConfig() const;
    void setSubscriberQueueSize(size_t queueSize);
    size_t getSubscriberQueueSize() const;

    void save(QSettings& settings) const;
    void load(QSettings& settings);
    void reset();
    
    void write(QDataStream& stream) const;
    void read(QDataStream& stream);
    
    CurveConfig& operator=(const CurveConfig& src);
    
  signals:
    void titleChanged(const QString& title);
    void subscriberQueueSizeChanged(size_t queueSize);
    
  private:
    QString title_;
    QMap<Axis, CurveAxisConfig*> axisConfig_;
    CurveColorConfig* colorConfig_;
    CurveStyleConfig* styleConfig_;
    CurveDataConfig* dataConfig_;
    size_t subscriberQueueSize_;
    
  private slots:
    void axisConfigChanged();
    void colorConfigChanged();
    void styleConfigChanged();
    void dataConfigChanged();
  };
};

#endif
