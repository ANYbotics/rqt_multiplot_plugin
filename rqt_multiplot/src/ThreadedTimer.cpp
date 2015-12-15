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

#include <QApplication>
#include <QTimerEvent>

#include "rqt_multiplot/ThreadedTimer.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ThreadedTimer::ThreadedTimer(QObject* parent) :
  QThread(parent),
  timer_(new QTimer()) {
  timer_->moveToThread(this);
  
  connect(timer_, SIGNAL(timeout()), this, SLOT(timerTimeout()));
}

ThreadedTimer::~ThreadedTimer() {
  quit();
  wait();
  
  delete timer_;
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

int ThreadedTimer::getTimerId() const {
  return timer_->timerId();
}

void ThreadedTimer::setRate(double rate) {
  timer_->setInterval(1e3/rate);
}

double ThreadedTimer::getRate() const {
  return 1e3/timer_->interval();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ThreadedTimer::run() {
  timer_->start();  

  QThread::exec();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ThreadedTimer::timerTimeout() {
  if (parent())
    QApplication::postEvent(parent(), new QTimerEvent(timer_->timerId()));
}

}
