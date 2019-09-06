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

#ifndef RQT_MULTIPLOT_MULTIPLOT_CONFIG_WIDGET_H
#define RQT_MULTIPLOT_MULTIPLOT_CONFIG_WIDGET_H

#include <QStringList>
#include <QWidget>

#include <rqt_multiplot/MultiplotConfig.h>

namespace Ui {
  class MultiplotConfigWidget;
};

namespace rqt_multiplot {
  class MultiplotConfigWidget :
    public QWidget {
  Q_OBJECT
  public:
    MultiplotConfigWidget(QWidget* parent = 0, size_t maxHistoryLength = 10);
    virtual ~MultiplotConfigWidget();

    void setConfig(MultiplotConfig* config);
    MultiplotConfig* getConfig() const;
    void setCurrentConfigUrl(const QString& url, bool updateHistory = true);
    QString getCurrentConfigUrl() const;
    bool setCurrentConfigModified(bool modified);
    bool isCurrentConfigModified() const;
    void setMaxConfigUrlHistoryLength(size_t length);
    size_t getMaxConfigUrlHistoryLength() const;
    void setConfigUrlHistory(const QStringList& history);
    QStringList getConfigUrlHistory() const;
    bool isFile(const QString& url) const;
    
    bool loadConfig(const QString& url);
    bool saveCurrentConfig();
    bool saveConfig(const QString& url);
    void resetConfig();
    
    bool confirmSave(bool canCancel = true);
    
    void addConfigUrlToHistory(const QString& url);
    void clearConfigUrlHistory();
    
  signals:
    void currentConfigModifiedChanged(bool modified);
    void currentConfigUrlChanged(const QString& url);
    
  private:
    Ui::MultiplotConfigWidget* ui_;
    
    MultiplotConfig* config_;
    
    QString currentConfigUrl_;
    bool currentConfigModified_;
    size_t maxHistoryLength_;
    
  private slots:
    void configChanged();
    
    void configComboBoxEditTextChanged(const QString& text);
    void configComboBoxCurrentUrlChanged(const QString& url);
    
    void pushButtonClearHistoryClicked();
    void pushButtonNewClicked();
    void pushButtonOpenClicked();
    void pushButtonSaveClicked();
    void pushButtonSaveAsClicked();
  };
};

#endif
