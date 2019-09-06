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

#include <QDir>

#include "rqt_multiplot/ConfigComboBox.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ConfigComboBox::ConfigComboBox(QWidget* parent) :
  UrlComboBox(parent),
  rootFileScheme_(new FileScheme(this)),
  homeFileScheme_(new FileScheme(this, "home", QDir::homePath())),
  packageScheme_(new PackageScheme(this)) {
  getCompleter()->getModel()->addScheme(rootFileScheme_);
  getCompleter()->getModel()->addScheme(homeFileScheme_);
  getCompleter()->getModel()->addScheme(packageScheme_);
  
  getCompleter()->setCompletionMode(QCompleter::InlineCompletion);
}

ConfigComboBox::~ConfigComboBox() {
}

}
