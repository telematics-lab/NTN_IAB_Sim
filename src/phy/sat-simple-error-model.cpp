/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2023 TELEMATICS LAB, Politecnico di Bari
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
 * Author: Daniele Pugliese <daniele.pugliese@poliba.it>
 */

#include "sat-simple-error-model.h"
#include "BLERTrace/BLERvsSINR_5G_SAT.h"
#include "../utility/RandomVariable.h"

bool
SatSimpleErrorModel::CheckForPhysicalError (vector<int> channels, vector<int> mcs, vector<double> sinr)
{
  /*
   * The device determines if a packet has been received correctly.
   * To this aim, for each sub-channel, used to transmit that packet,
   * the Block Error Rate (BLER) is estimated.
   *
   * The BLER is obtained considering both MCS used for the transmission
   * and SINR that the device has estimated for the considered sub-channel.
   * In particular, the BLER value is drawn using stored BLER-SINR curves
   * stored into trace files, generated using the MATLAB 5G Toolbox.
   * According to the proper BLER-SINR curve (depending on the used MCS),
   * the device establishes if the packet has been correctly received or not.
   * In the latter case, the packet is considered erroneous and ignored.
   */

  bool error = false;
  double randomNumber = (rand () %100 ) / 100.;
  if(mcs.size()<1){
   return error;
  }
  int mcs_ = mcs.at(0);
  for (int i = 0; i < (int)channels.size (); i++)
    {

      double sinr_ = sinr.at (i);
      double bler;

      bler = GetBLER_SAT5G (sinr_, mcs_);

      if (randomNumber < bler)
        {
          error = true;
          if (_TEST_BLER_) cout << "BLER PDF " << sinr_ << " 1" << endl;
        }
      else
        {
          if (_TEST_BLER_) cout << "BLER PDF " << sinr_ << " 0" << endl;
        }
    }

  return error;
}


