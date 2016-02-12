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

#ifndef RQT_MULTIPLOT_STATUS_WIDGET_H
#define RQT_MULTIPLOT_STATUS_WIDGET_H

#include <QGridLayout>
#include <QImage>
#include <QLabel>
#include <QList>
#include <QMap>
#include <QPixmap>
#include <QTimer>
#include <QWidget>

namespace rqt_multiplot {
  class StatusWidget :
    public QWidget {
  Q_OBJECT
  public:
    enum Role {
      Okay,
      Error,
      Busy
    };

    StatusWidget(QWidget* parent = 0, Role role = Okay);
    virtual ~StatusWidget();

    void setIcon(Role role, const QPixmap& icon);
    const QPixmap& getIcon(Role role) const;
    void setFrames(Role role, const QPixmap& frames, size_t numFrames,
      double frameRate = 10.0);
    void setFrames(Role role, const QList<QPixmap>& frameList, double
      frameRate = 10.0);
    const QList<QPixmap>& getFrames(Role role) const;
    void setFrameRate(Role role, double frameRate);
    double getFrameRate(Role role) const;
    void setCurrentRole(Role role, const QString& toolTip = QString());
    Role getCurrentRole() const;

    void pushCurrentRole();
    bool popCurrentRole();

  signals:
    void currentRoleChanged(Role role);

  private:
    QGridLayout* layout_;
    QLabel* labelIcon_;
    QTimer* timer_;

    QMap<Role, QList<QPixmap> > frames_;
    QMap<Role, double> frameRates_;
    QList<Role> roleStack_;
    QList<QString> toolTipStack_;

    Role currentRole_;
    size_t currentFrame_;

    void start();
    void step();
    void stop();

  private slots:
    void timerTimeout();
  };
};

#endif
