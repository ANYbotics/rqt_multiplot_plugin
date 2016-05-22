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

#include <QChildEvent>
#include <QDrag>
#include <QEvent>
#include <QMimeData>
#include <QMouseEvent>

#include <qwt/qwt.h>
#include <qwt/qwt_dyngrid_layout.h>
#if QWT_VERSION >= 0x060100
  #include <qwt/qwt_legend_label.h>
  #include <qwt/qwt_plot_legenditem.h>
#else
  #include <qwt/qwt_legend_item.h>
  #include <qwt/qwt_legend_itemmanager.h>
#endif

#include <rqt_multiplot/PlotCurve.h>
#include <rqt_multiplot/PlotWidget.h>
#include <rqt_multiplot/CurveConfigDialog.h>
#include <rqt_multiplot/CurveConfigWidget.h>

#include "rqt_multiplot/PlotLegend.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PlotLegend::PlotLegend(QWidget* parent) :
  QwtLegend(parent) {
  QwtDynGridLayout* layout = static_cast<QwtDynGridLayout*>(
    contentsWidget()->layout());
  layout->setSpacing(10);
}

PlotLegend::~PlotLegend() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

PlotCurve* PlotLegend::findCurve(QWidget* widget) const {
  #if QWT_VERSION >= 0x060100
    QVariant info = itemInfo(widget);

    if (info.canConvert<QwtPlotItem*>())
      return dynamic_cast<PlotCurve*>(info.value<QwtPlotItem*>());
    else
      return 0;
  #else
    QwtLegendItemManager* legendItemManager = find(widget);

    if (legendItemManager)
      return dynamic_cast<PlotCurve*>(legendItemManager);
    else
      return 0;
  #endif
}

bool PlotLegend::eventFilter(QObject* object, QEvent* event) {
  if (object == contentsWidget()) {
    if (event->type() == QEvent::ChildAdded) {
      QChildEvent* childEvent = static_cast<QChildEvent*>(event);

      #if QWT_VERSION >= 0x060100
        QwtLegendLabel* legendItem = qobject_cast<QwtLegendLabel*>(
          childEvent->child());
      #else
        QwtLegendItem* legendItem = qobject_cast<QwtLegendItem*>(
          childEvent->child());
      #endif

      if (legendItem) {
        legendItem->setCursor(Qt::PointingHandCursor);
        legendItem->installEventFilter(this);
      }
    }
  }
  else if (object->isWidgetType()) {
    QWidget* widget = static_cast<QWidget*>(object);
    PlotCurve* curve = findCurve(widget);

    if (curve && curve->getConfig()) {
      if (event->type() == QEvent::MouseButtonDblClick) {
        CurveConfig* curveConfig = curve->getConfig();
        CurveConfigDialog dialog(this);

        dialog.setWindowTitle(curveConfig->getTitle().isEmpty() ?
          "Edit Curve" :
          "Edit \""+curveConfig->getTitle()+"\"");
        dialog.getWidget()->setConfig(*curveConfig);

        if (dialog.exec() == QDialog::Accepted)
          *curveConfig = dialog.getWidget()->getConfig();
      }
      else if (event->type() == QEvent::MouseButtonPress) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

        if ((mouseEvent->button() == Qt::LeftButton) ||
            (mouseEvent->button() == Qt::RightButton)) {
          QByteArray data;
          QDataStream stream(&data, QIODevice::WriteOnly);
          stream << *curve->getConfig();

          QMimeData* mimeData = new QMimeData();
          mimeData->setData(CurveConfig::MimeType, data);

          QPixmap pixmap(widget->size());
          pixmap.fill(Qt::transparent);
          widget->render(&pixmap, QPoint(), QRegion(), QWidget::DrawChildren);

          QPoint hotSpot = mouseEvent->pos();
          hotSpot.setX(0.5*pixmap.width());
          hotSpot.setY(pixmap.height()+5);

          QDrag* drag = new QDrag(this);
          drag->setMimeData(mimeData);
          drag->setPixmap(pixmap);
          drag->setHotSpot(hotSpot);

          Qt::DropAction defaultDropAction = Qt::CopyAction;
          if (mouseEvent->button() == Qt::RightButton)
            defaultDropAction = Qt::MoveAction;

          Qt::DropAction dropAction = drag->exec(Qt::CopyAction |
            Qt::MoveAction, defaultDropAction);

          if (dropAction == Qt::MoveAction)
            curve->getConfig()->deleteLater();
        }
      }
    }
  }

  return QwtLegend::eventFilter(object, event);
}

}
