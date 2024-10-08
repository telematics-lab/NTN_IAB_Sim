/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of 5G-air-simulator
 *
 * 5G-air-simulator is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * 5G-air-simulator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 5G-air-simulator; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Telematics Lab <telematics-dev@poliba.it>
 */


#ifndef RANDOMVARIABLE_H_
#define RANDOMVARIABLE_H_

#include <stdint.h>
#include "stdlib.h"

static double lastValue = 0;

static double
GetRandomVariable (int seed, double maxValue)
{
  lastValue =  rand() * maxValue / RAND_MAX;
  return lastValue;
}


static double
GetRandomVariable (double maxValue)
{
  lastValue = rand() * maxValue / RAND_MAX;
  return lastValue;
}

#endif /* RANDOMVARIABLE_H_ */
