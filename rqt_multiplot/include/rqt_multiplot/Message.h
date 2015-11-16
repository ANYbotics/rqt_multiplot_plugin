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

#ifndef RQT_MULTIPLOT_MESSAGE_H
#define RQT_MULTIPLOT_MESSAGE_H

#include <ros/time.h>

#include <variant_topic_tools/MessageVariant.h>

namespace rqt_multiplot {
  class Message {
  public:
    Message();
    Message(const Message& src);
    ~Message();
    
    void setReceiptTime(const ros::Time& receiptTime);
    const ros::Time& getReceiptTime() const;  
    void setVariant(const variant_topic_tools::MessageVariant& variant);
    const variant_topic_tools::MessageVariant& getVariant() const;
    bool isEmpty() const;
    
  private:
    ros::Time receiptTime_;
    variant_topic_tools::MessageVariant variant_;
  };
};

#endif
