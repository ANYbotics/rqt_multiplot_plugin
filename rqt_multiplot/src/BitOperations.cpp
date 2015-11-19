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

#include <limits.h>

#include "rqt_multiplot/BitOperations.h"

namespace rqt_multiplot {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

unsigned int BitOperations::revertInt(unsigned int val) {
  unsigned int reverse = val;
  unsigned int shift = sizeof(unsigned int)*CHAR_BIT-1;

  for (val >>= 1; val; val >>= 1) {
    reverse <<= 1;
    reverse |= val & 1;
    --shift;
  }
  reverse <<= shift;

  return reverse;
}
               
unsigned short BitOperations::revertShort(unsigned short val) {
  unsigned short reverse = val;
  unsigned short shift = sizeof(unsigned short)*CHAR_BIT-1;

  for (val >>= 1; val; val >>= 1) {
    reverse <<= 1;
    reverse |= val & 1;
    --shift;
  }
  reverse <<= shift;

  return reverse;
}

unsigned char BitOperations::revertByte(unsigned char val) {
  unsigned char reverse = val;
  unsigned char shift = sizeof(unsigned char)*CHAR_BIT-1;

  for (val >>= 1; val; val >>= 1) {
    reverse <<= 1;
    reverse |= val & 1;
    --shift;
  }
  reverse <<= shift;

  return reverse;
}

};
