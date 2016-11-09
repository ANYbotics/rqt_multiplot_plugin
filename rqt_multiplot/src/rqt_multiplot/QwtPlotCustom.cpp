#include "rqt_multiplot/QwtPlotCustom.h"

namespace rqt_multiplot {

QwtPlotCustom::QwtPlotCustom(QWidget *parent)
    : QwtPlot(parent) {

}

QwtPlotCustom::QwtPlotCustom(const QwtText &title, QWidget *p)
    : QwtPlot(title, p) {

}

QSize QwtPlotCustom::sizeHint() const {
  return QSize(50, 50);
}

QSize QwtPlotCustom::minimumSizeHint() const {
  return QSize(50, 50);
}

}
