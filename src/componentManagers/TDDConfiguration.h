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



#ifndef TDDCONFIGURATION_H_
#define TDDCONFIGURATION_H_

/*
 * Frame structure type 2 is applicable to TDD [TS 36.211].
 * Each radio frame consists of two half-frames.
 * Each half-frame consists of five subframes of length .
 */
enum SubFrameType
{
  SUBFRAME_FOR_DOWNLINK,  // code 0
  SUBFRAME_FOR_UPLINK,  // code 1
  SPECIAL_SUBFRAME,    // code 2
  SUBFRAME_FOR_DOWNLINK_IAB,  // code 3
  SUBFRAME_FOR_UPLINK_IAB,  // code 4
  SUBFRAME_FOR_BACKHAUL_FDD,  // code 5
};

static int TDDConfiguration[12][10] =
    {
        {0, 2, 1, 1, 1, 0, 2, 1, 1, 1}, // 0

        {0, 2, 1, 1, 0, 0, 2, 1, 1, 0}, // 1

        {0, 2, 1, 0, 0, 0, 2, 1, 0, 0}, // 2

        {0, 2, 1, 1, 1, 0, 0, 0, 0, 0}, // 3

        {0, 2, 1, 1, 0, 0, 0, 0, 0, 0}, // 4

        {0, 2, 1, 0, 0, 0, 0, 0, 0, 0}, // 5

        {0, 2, 1, 1, 1, 0, 2, 1, 1, 0}, // 6

        {1, 0, 2, 4, 4, 4, 3, 3, 3, 2}, // 7

        {1, 1, 0, 0, 2, 4, 4, 3, 3, 2}, // 8

        {2, 1, 1, 1, 0, 0, 0, 2, 4, 3}, // 9

        {5, 5, 5, 5, 2, 4, 4, 3, 3, 2}, // 10

        {5, 5, 5, 5, 5, 5, 5, 5, 5, 5}  // 11
};

#endif /* TDDCONFIGURATION_H_ */
