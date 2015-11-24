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

#ifndef RQT_MULTIPLOT_CURVE_COLOR_H
#define RQT_MULTIPLOT_CURVE_COLOR_H

#include <QColor>
#include <QObject>
#include <QSettings>

namespace rqt_multiplot {
  class CurveColor :
    public QObject {
  Q_OBJECT
  public:
    enum Type {
      Auto,
      Custom
    };
    
    CurveColor(QObject* parent, Type type = Auto, unsigned char
      autoColorIndex = 0, const QColor& customColor = Qt::black);
    ~CurveColor();
    
    void setType(Type type);
    Type getType() const;
    void setAutoColorIndex(unsigned char index);
    unsigned char getAutoColorIndex() const;
    void setCustomColor(const QColor& color);
    const QColor& getCustomColor() const;
    QColor getCurrentColor() const;
  
    void save(QSettings& settings) const;
    void load(QSettings& settings);
    
    CurveColor& operator=(const CurveColor& src);
    
  signals:
    void typeChanged(int type);
    void autoColorIndexChanged(unsigned char index);
    void customColorChanged(const QColor& color);
    void currentColorChanged(const QColor& color);
    void changed();
    
  private:
    Type type_;
    unsigned char autoColorIndex_;
    QColor customColor_;
  };
};

#endif
