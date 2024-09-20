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


#include "NetworkNode.h"
#include "UserEquipment.h"
#include "GNodeB.h"
#include "IABNode.h"
#include "Gateway.h"
#include "../protocolStack/mac/packet-scheduler/packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/mt-uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-pf-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-mt-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-mlwdf-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-exp-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-fls-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/exp-rule-downlink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/log-rule-downlink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-rr-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/enhanced-uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/roundrobin-uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/nb-roundrobin-uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/nb-fifo-uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/nb-uplink-packet-scheduler.h"
#include "../phy/gnb-phy.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../protocolStack/packet/packet-burst.h"
#include "../protocolStack/mac/harq-manager.h"
#include "../componentManagers/FrameManager.h"
#include "../protocolStack/mac/random-access/gnb-random-access.h"

IABNode::IABNode (int idElement,
                Cell *cell)
    : IABNode::IABNode (idElement,
                      cell,
                      cell->GetCellCenterPosition ()->GetCoordinateX (),
                      cell->GetCellCenterPosition ()->GetCoordinateY (),
                      25) // default value for urban macro-cell scenario
{}

IABNode::IABNode (int idElement,
                Cell *cell,
                double posx,
                double posy)
    : IABNode::IABNode (idElement,
                      cell,
                      posx,
                      posy,
                      25) // default value for urban macro-cell scenario
{}

IABNode::IABNode (int idElement,
                Cell *cell,
                double posx,
                double posy,
                double posz)
{
  SetIDNetworkNode (idElement);
  SetNodeType(NetworkNode::TYPE_RELAY_NODE);
  SetCell (cell); // Set the cell for this IABNode

  //Fixed position for the IABNode (for the moment)
  CartesianCoordinates *position = new CartesianCoordinates(posx, posy, posz);
  Mobility* m = new ConstantPosition (); 
  m->SetAbsolutePosition (position);
  SetMobilityModel (m);
  delete position;

  m_userEquipmentRecords = new UserEquipmentRecords;

  GnbPhy *phy = new GnbPhy ();
  phy->SetDevice(this);
  SetPhy (phy);

  ProtocolStack *stack = new ProtocolStack (this);
  SetProtocolStack (stack);

  Classifier *classifier = new Classifier ();
  classifier->SetDevice (this);
  SetClassifier (classifier);
}

IABNode::IABNode(int idElement,
               Cell *cell,
               double posx,
               double posy,
               double posz,
               int nSat, bool isNBIoT) //Constructor for satellite scenario
{

  SetIDNetworkNode(idElement);
  SetNodeType(NetworkNode::TYPE_RELAY_NODE);
  SetCell(cell); // Set the cell for this IABNode

  CartesianCoordinates *position = new CartesianCoordinates(posx, posy, posz);
  Mobility *m;
  if (isNBIoT)
  {
    m = new SatelliteMovement(nSat, true); // movimento satellitare
  }
  else
  {
    m = new SatelliteMovement(nSat, false); // movimento satellitare
  }
  m->SetAbsolutePosition(position);
  SetMobilityModel(m);
  m->SetDevice(this);
  delete position;

  m_userEquipmentRecords = new UserEquipmentRecords;

  GnbPhy *phy = new GnbPhy();
  phy->SetDevice(this);
  SetPhy(phy);

  ProtocolStack *stack = new ProtocolStack(this);
  SetProtocolStack(stack);

  Classifier *classifier = new Classifier();
  classifier->SetDevice(this);
  SetClassifier(classifier);
}

IABNode::~IABNode()
{
  Destroy ();
  DeleteUserEquipmentRecords ();
}
void
IABNode::SetMobileTermination (UserEquipment* mt)
{
  m_mobileTermination = mt;
  //SetCell (n->GetCell ());
}

UserEquipment*
IABNode::GetMobileTermination (void)
{
  return m_mobileTermination;
}