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

#include "ue-satellite.h"
#include "../componentManagers/NetworkManager.h"
#include "../device/Gateway.h"
#include "../device/GNodeB.h"
#include "../device/UserEquipment.h"
#include "../load-parameters.h"

UeSatelliteMovement::UeSatelliteMovement()
{
  SetMobilityModel(Mobility::UE_SATELLITE);
  SetSpeed (0);
  SetSpeedDirection (0.0);
  SetPositionLastUpdate (0.0);
  SetHandover (false); //??
  SetLastHandoverTime (0.0);
  SetAbsolutePosition (nullptr);
}

UeSatelliteMovement::~UeSatelliteMovement()
{
  DeleteAbsolutePosition ();
}

void UeSatelliteMovement::UpdatePosition(double time)
{
  UserEquipment *thisNode = (UserEquipment*)GetDevice();
  if (thisNode->GetTargetNode()->GetMobilityModel()->GetMobilityModel() != Mobility::SATELLITE)
  {
    IABNode *iab = dynamic_cast<UserEquipment *>(thisNode)->GetIABNode();
    SetAbsolutePosition(iab->GetMobilityModel()->GetAbsolutePosition());
    SetPositionLastUpdate(time);
    Simulator::Init()->Schedule(0.001,
                                &UeSatelliteMovement::UpdatePosition,
                                this,
                                Simulator::Init()->Now());
  }
  else
  {
    SetPositionLastUpdate(time);
  }
}

void
UeSatelliteMovement::RefreshTimePositionUpdate ()
{
    UserEquipment *thisNode = (UserEquipment*)GetDevice();
    if (thisNode->GetTargetNode()->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE) {
        double t = ((SatelliteMovement*) thisNode->GetTargetNode()->GetMobilityModel())->GetNextTimePositionUpdate(GetAbsolutePosition()) ;
        thisNode->SetTimePositionUpdate(t);
    }
    else{
        cout << "Wrong mobility models selected";
        exit(1);
    }
}
