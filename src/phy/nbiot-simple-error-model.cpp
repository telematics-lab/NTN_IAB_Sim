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
 * Author: Antonio Petrosino  <antonio.petrosino@poliba.it>
 * Author: Giancarlo Sciddurlo  <giancarlo.sciddurlo@poliba.it>
 */

#include "nbiot-simple-error-model.h"
#include "BLERTrace/BLERvsSINR_NBIoT_SAT.h"
#include "../utility/RandomVariable.h"

bool
NBIoTSimpleErrorModel::CheckForPhysicalError (vector<int> channels, vector<int> mcs, vector<double> sinr)
{
  /*
   * The device determines if a packet has been received correctly.
   * To this aim, for each sub-channel, used to transmit that packet,
   * the Block Error Rate (BLER) is estimated.
   *
   * The BLER is obtained considering both MCS used for the transmission
   * and SINR that the device has estimated for the considered sub-channel.
   * In particular, the BLER value is drawn using stored BLER-SINR curves
   * stored into trace files, generated using an ad hoc OFDMA tool written in matlab.
   * According to the proper BLER-SINR curve (depending on the used MCS),
   * the device establishes if the packet has been correctly received or not.
   * In the latter case, the packet is considered erroneous and ignored.
   */

  bool error = false;
  double randomNumber = (rand () %100 ) / 100.;

  for (int i = 0; i < (int)channels.size (); i++)
    {
      //int mcs_ = FrameManager::Init()->GetMCSNBIoTSat ();
	  int mcs_ = mcs[0];

      double sinr_ = sinr.at (channels.at (i));
      double bler;

      bler = GetBLER_SAT (sinr_, mcs_);

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

bool
NBIoTSimpleErrorModel::CheckForPhysicalErrorHARQ (vector<int> channels, vector<double> sinr)
{

  bool error = false;
  double randomNumber = (rand () %100 ) / 100.;

  for (int i = 0; i < (int)channels.size (); i++)
    {
      //int mcs_ = FrameManager::Init()->GetMCSNBIoTSat ();
	  //int mcs_ = mcs[0];

      double sinr_ = sinr.at (channels.at (i));
      double bler;

      bler = GetBlerHARQ (sinr_);

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

int
NBIoTSimpleErrorModel::GetRefMCS(int MCS, int NRU){
	double coderate = 0.0;

	if (MCS > 0 && MCS <= 10){
		if(NRU > 0 &&  NRU <= 5 ){
			coderate = CodeRatesSingleToneTable[MCS][NRU-1];
		}
	}

	double minOffset = 9999.99;
	int newMCSindex = 0;
	//int newNRUindex = 0;

	for(int i=0; i<5; i++){
		if((abs(coderate - RefCodeRates[i])) < minOffset){
			minOffset = abs(coderate - RefCodeRates[i]);
			newMCSindex = RefMCS[i];
			//newNRUindex = RefNRU[i];
		}
	}

	return newMCSindex;
	// since TBS is constant for each BLER curves stored
	// we need only one MCS value to find the corresponding curve
}
