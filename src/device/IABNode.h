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


#ifndef IABNODE_H_
#define IABNODE_H_

#include "NetworkNode.h"
#include "GNodeB.h"
#include "UserEquipment.h"
#include <memory>
#include <armadillo>

//class UserEquipment;
//class Gateway;

//class PacketScheduler;
//class DownlinkPacketScheduler;
//class UplinkPacketScheduler;

class IABNode : public GNodeB
{
public:
    IABNode() = default;
    IABNode(int idElement, Cell *cell);
    IABNode(int idElement, Cell *cell, double posx, double posy);
    IABNode(int idElement, Cell *cell, double posx, double posy, double posz);
    IABNode(int idElement, Cell *cell, double posx, double posy, double posz, int nSat, bool isNBIoT);
      virtual ~IABNode();
      void SetMobileTermination(UserEquipment * mt);
      UserEquipment *GetMobileTermination(void);

    private:
      UserEquipment *m_mobileTermination;


};

#endif /* IABNODE_H_ */