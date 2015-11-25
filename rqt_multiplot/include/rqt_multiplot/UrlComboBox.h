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

#ifndef RQT_MULTIPLOT_URL_COMBO_BOX_H
#define RQT_MULTIPLOT_URL_COMBO_BOX_H

#include <QComboBox>

#include <rqt_multiplot/UrlCompleter.h>

namespace rqt_multiplot {
  class UrlComboBox :
    public QComboBox {
  Q_OBJECT
  public:
    UrlComboBox(QWidget* parent = 0);
    virtual ~UrlComboBox();
    
    void setEditable(bool editable);
    UrlCompleter* getCompleter() const;
    void setCurrentUrl(const QString& url);
    QString getCurrentUrl() const;
    bool isCurrentUrlSelectable() const;
    
  signals:
    void currentUrlChanged(const QString& url);
    
  private:
    QString currentUrl_;
    
    UrlCompleter* completer_;
    
  private slots:
    void activated(int index);
    void currentIndexChanged(const QString& text);
    void lineEditEditingFinished();
  };
};

#endif
