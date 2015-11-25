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

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QSettings>

#include <ros/package.h>

#include <ui_MultiplotConfigWidget.h>

#include "rqt_multiplot/MultiplotConfigWidget.h"
#include "rqt_multiplot/XmlSettings.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MultiplotConfigWidget::MultiplotConfigWidget(QWidget* parent, size_t
    maxHistoryLength) :
  QWidget(parent),
  ui_(new Ui::MultiplotConfigWidget()),
  config_(0),
  currentConfigModified_(false),
  maxHistoryLength_(maxHistoryLength) {
  ui_->setupUi(this);
  
  ui_->pushButtonNew->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/add.png"))));
  ui_->pushButtonOpen->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/open.png"))));
  ui_->pushButtonSave->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/save.png"))));
  ui_->pushButtonSaveAs->setIcon(
    QIcon(QString::fromStdString(ros::package::getPath("rqt_multiplot").
    append("/resource/16x16/save_as.png"))));
  
  ui_->pushButtonSave->setEnabled(false);
  
  connect(ui_->configComboBox, SIGNAL(editTextChanged(const QString&)),
    this, SLOT(configComboBoxEditTextChanged(const QString&)));
  connect(ui_->configComboBox, SIGNAL(currentUrlChanged(const QString&)),
    this, SLOT(configComboBoxCurrentUrlChanged(const QString&)));
  
  connect(ui_->pushButtonNew, SIGNAL(clicked()), this,
    SLOT(pushButtonNewClicked()));
  connect(ui_->pushButtonOpen, SIGNAL(clicked()), this,
    SLOT(pushButtonOpenClicked()));
  connect(ui_->pushButtonSave, SIGNAL(clicked()), this,
    SLOT(pushButtonSaveClicked()));
  connect(ui_->pushButtonSaveAs, SIGNAL(clicked()), this,
    SLOT(pushButtonSaveAsClicked()));
}

MultiplotConfigWidget::~MultiplotConfigWidget() {
  delete ui_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MultiplotConfigWidget::setConfig(MultiplotConfig* config) {
  if (config != config_) {
    if (config_)
      disconnect(config_, SIGNAL(changed()), this, SLOT(configChanged()));
    
    config_ = config;
    
    if (config)
      connect(config_, SIGNAL(changed()), this, SLOT(configChanged()));
  }
}

MultiplotConfig* MultiplotConfigWidget::getConfig() const {
  return config_;
}

void MultiplotConfigWidget::setCurrentConfigUrl(const QString& url, bool
    updateHistory) {
  if (url != currentConfigUrl_) {
    currentConfigUrl_ = url;
    
    if (updateHistory)
      addConfigUrlToHistory(url);
    
    ui_->configComboBox->setCurrentUrl(url);
    
    emit currentConfigUrlChanged(url);
  }  
}

QString MultiplotConfigWidget::getCurrentConfigUrl() const {
  return ui_->configComboBox->getCurrentUrl();
}

bool MultiplotConfigWidget::setCurrentConfigModified(bool modified) {
  if (modified != currentConfigModified_) {
    currentConfigModified_ = modified;
    
    ui_->pushButtonSave->setEnabled(!currentConfigUrl_.isEmpty() &&
      (ui_->configComboBox->getCurrentUrl() == currentConfigUrl_) &&
      modified);
    
    emit currentConfigModifiedChanged(modified);
  }
}

bool MultiplotConfigWidget::isCurrentConfigModified() const {
  return currentConfigModified_;
}

void MultiplotConfigWidget::setMaxConfigUrlHistoryLength(size_t length) {
  if (length != maxHistoryLength_) {
    maxHistoryLength_ = length;
    
    while (ui_->configComboBox->count() > length)
      ui_->configComboBox->removeItem(ui_->configComboBox->count()-1);
  }
}

size_t MultiplotConfigWidget::getMaxConfigUrlHistoryLength() const {
  return maxHistoryLength_;
}

void MultiplotConfigWidget::setConfigUrlHistory(const QStringList& history) {
  ui_->configComboBox->clear();
  
  for (size_t i = 0; (i < history.count()) && (i < maxHistoryLength_); ++i)
    ui_->configComboBox->addItem(history[i]);
}

QStringList MultiplotConfigWidget::getConfigUrlHistory() const {
  QStringList history;
  
  for (size_t i = 0; i < ui_->configComboBox->count(); ++i)
    history.append(ui_->configComboBox->itemText(i));
  
  return history;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool MultiplotConfigWidget::loadConfig(const QString& url) {
  if (config_) {
    UrlItemModel* model = ui_->configComboBox->getCompleter()->getModel();
    QString filePath = model->getFilePath(url);
    
    if (!filePath.isEmpty()) {
      QFileInfo fileInfo(filePath);
      
      if (fileInfo.isReadable()) {
        QSettings settings(filePath, XmlSettings::format);
        
        if (settings.status() == QSettings::NoError) {
          settings.beginGroup("rqt_multiplot");
          config_->load(settings);
          settings.endGroup();
          
          setCurrentConfigUrl(url);
          setCurrentConfigModified(false);
          
          return true;
        }
      }
    }
  }

  return false;
}

bool MultiplotConfigWidget::saveCurrentConfig() {
  if (!currentConfigUrl_.isEmpty())
    return saveConfig(currentConfigUrl_);
    
  return false;
}

bool MultiplotConfigWidget::saveConfig(const QString& url) {
  if (config_) {
    UrlItemModel* model = ui_->configComboBox->getCompleter()->getModel();
    QString filePath = model->getFilePath(url);
    
    if (!filePath.isEmpty()) {
      QSettings settings(filePath, XmlSettings::format);

      if (settings.isWritable()) {
        settings.clear();
        
        settings.beginGroup("rqt_multiplot");
        config_->save(settings);
        settings.endGroup();
        
        settings.sync();
        
        if (settings.status() == QSettings::NoError) {
          setCurrentConfigUrl(url);
          setCurrentConfigModified(false);
          
          return true;
        }
      }
    }
  }
  
  return false;
}

void MultiplotConfigWidget::resetConfig() {
  if (config_) {
    config_->reset();
    
    setCurrentConfigUrl(QString());
    setCurrentConfigModified(false);
  }
}

bool MultiplotConfigWidget::confirmSave() {
  if (currentConfigModified_) {
    QMessageBox messageBox;
    
    messageBox.setText("The configuration has been modified.");
    messageBox.setInformativeText("Do you want to save your changes?");
    messageBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard |
      QMessageBox::Cancel);
    messageBox.setDefaultButton(QMessageBox::Save);
    
    switch (messageBox.exec()) {
      case QMessageBox::Save:
        if (currentConfigUrl_.isEmpty()) {
          QFileDialog dialog(this, "Save Configuration", QDir::homePath(),
            "Multiplot configurations (*.xml)");
          
          dialog.setAcceptMode(QFileDialog::AcceptSave);
          dialog.setFileMode(QFileDialog::AnyFile);
          dialog.selectFile("rqt_multiplot.xml");
          
          if (dialog.exec() == QDialog::Accepted)
            return saveConfig("file://"+dialog.selectedFiles().first());
          else
            return false;
        }
        else
          return saveCurrentConfig();
      case QMessageBox::Discard:
        return true;
      case QMessageBox::Cancel:
        return false;
      default:
        return false;
    }
  }
  
  return true;
}

void MultiplotConfigWidget::addConfigUrlToHistory(const QString& url) {
  if (!url.isEmpty()) {
    int index = ui_->configComboBox->findText(url);
    
    ui_->configComboBox->blockSignals(true);
    
    if (index < 0) {
      while (ui_->configComboBox->count()+1 > maxHistoryLength_)
        ui_->configComboBox->removeItem(ui_->configComboBox->count()-1);      
    }
    else
      ui_->configComboBox->removeItem(index);
    
    ui_->configComboBox->insertItem(0, url);
    
    ui_->configComboBox->blockSignals(false);
  }
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void MultiplotConfigWidget::configChanged() {
  setCurrentConfigModified(true);
}

void MultiplotConfigWidget::configComboBoxEditTextChanged(const
    QString& text) {
  ui_->pushButtonSave->setEnabled(!currentConfigUrl_.isEmpty() &&
    (text == currentConfigUrl_) && currentConfigModified_);
}

void MultiplotConfigWidget::configComboBoxCurrentUrlChanged(const
    QString& url) {
  if (url != currentConfigUrl_)
    loadConfig(url);
  
  ui_->pushButtonSave->setEnabled(!currentConfigUrl_.isEmpty() &&
    (url == currentConfigUrl_) && currentConfigModified_);
}

void MultiplotConfigWidget::pushButtonNewClicked() {
  if (confirmSave())
    resetConfig();
}

void MultiplotConfigWidget::pushButtonOpenClicked() {
  if (confirmSave()) {
    QFileDialog dialog(this, "Open Configuration", QDir::homePath(),
      "Multiplot configurations (*.xml)");
    
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    dialog.setFileMode(QFileDialog::ExistingFile);
    
    if (dialog.exec() == QDialog::Accepted)
      loadConfig("file://"+dialog.selectedFiles().first());
  }
}

void MultiplotConfigWidget::pushButtonSaveClicked() {
  saveCurrentConfig();
}

void MultiplotConfigWidget::pushButtonSaveAsClicked() {
  QFileDialog dialog(this, "Save Configuration", QDir::homePath(),
    "Multiplot configurations (*.xml)");
  
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  dialog.setFileMode(QFileDialog::AnyFile);
  dialog.selectFile("rqt_multiplot.xml");
  
  if (dialog.exec() == QDialog::Accepted)
    saveConfig("file://"+dialog.selectedFiles().first());
}

}
