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

#ifndef RQT_MULTIPLOT_URL_ITEM_MODEL_H
#define RQT_MULTIPLOT_URL_ITEM_MODEL_H

#include <QAbstractItemModel>
#include <QList>

#include <rqt_multiplot/UrlItem.h>
#include <rqt_multiplot/UrlScheme.h>

namespace rqt_multiplot {
class UrlItemModel : public QAbstractItemModel {
  Q_OBJECT
 public:
  explicit UrlItemModel(QObject* parent = nullptr);
  ~UrlItemModel() override;

  static QString getUrl(const QModelIndex& index);
  static QString getFilePath(const QModelIndex& index);
  QString getFilePath(const QString& url) const;
  static UrlScheme* getScheme(const QModelIndex& index);

  void addScheme(UrlScheme* scheme);

  int rowCount(const QModelIndex& parent) const override;
  int columnCount(const QModelIndex& parent) const override;
  QVariant data(const QModelIndex& index, int role) const override;
  QModelIndex index(int row, int column, const QModelIndex& parent) const override;
  QModelIndex parent(const QModelIndex& index) const override;

 signals:
  void urlLoaded(const QString& url);

 private:
  QList<UrlScheme*> schemes_;
  QList<UrlItem*> schemeItems_;

 private slots:
  void schemeResetStarted();
  void schemeResetFinished();
  void schemePathLoaded(const QString& host, const QString& path);
};
}  // namespace rqt_multiplot

#endif
