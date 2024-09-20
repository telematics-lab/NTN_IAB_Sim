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

#include "../channel/RadioChannel.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../networkTopology/Cell.h"
#include "../core/eventScheduler/simulator.h"
#include "../flows/application/InfiniteBuffer.h"
#include "../flows/QoS/QoSParameters.h"
#include "../componentManagers/FrameManager.h"
#include "../componentManagers/FlowsManager.h"
#include "../device/GNodeB.h"
#include "../device/IABNode.h"
#include "../phy/gnb-phy.h"
#include "../phy/ue-phy.h"
#include "../phy/wideband-cqi-eesm-error-model.h"
#include "../protocolStack/packet/packet-burst.h"
#include "../protocolStack/packet/Packet.h"
#include "../device/NetworkNode.h"
#include "../device/UserEquipment.h"
#include "../device/Gateway.h"
#include "../load-parameters.h"
#include "../device/CqiManager/fullband-cqi-manager.h"
#include "../device/CqiManager/wideband-cqi-manager.h"
#include "../channel/propagation-model/channel-realization.h"
#include "../channel/propagation-model/propagation-loss-model.h"
#include "../utility/CellPosition.h"
#include "../flows/application/CBR.h"
#include "../flows/application/relay-sender.h"

static void RelayExampleDl()
{
  /*
  The following Scenario simulate the case in wich a GNodeB (ID = 4) try to periodically send packets
  through a CBR Application, being in the range of a MobileTermination (ID = 3) in charge to receive packets and relays
  them via an IABNode (ID = 2) to a UE (ID = 1).
  */

  // CREATE COMPONENT MANAGERS
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager *networkManager = NetworkManager::Init();
  FlowsManager *flowsManager = FlowsManager::Init();

  // CONFIGURE SEED
  int seed = time(nullptr);
  srand(seed);

  // SET RANDOM ACCESS //Maybe not necessary for downlink case
  UeRandomAccess::RandomAccessType m_UeRandomAccessType = UeRandomAccess::RA_TYPE_BASELINE;
  GnbRandomAccess::RandomAccessType m_GnbRandomAccessType = GnbRandomAccess::RA_TYPE_BASELINE;

  // SET DUPLEXING MODE
  //frameManager->SetFrameStructure(FrameManager::FRAME_STRUCTURE_FDD); // Default value
  frameManager->SetFrameStructure(FrameManager::FRAME_STRUCTURE_TDD);
  frameManager->SetTDDFrameConfiguration(8);
  
  // CREATE CELL
  int idCell = 0;
  int radius = 1;           // km
  int minDistance = 0.0035; // km
  int posX = 0;
  int posY = 0;
  Cell *cell = networkManager->CreateCell(idCell, radius, minDistance, posX, posY);

  // CREATE CHANNELS AND SPECTRUM

  // The two different link must use different frequencies
  int spectrum_index = 0;
  int offset = 100; //Note: The logic is derived from RunFrequencyReuseTechniques method

  // MT-gNB
  RadioChannel *dlCh = new RadioChannel();
  RadioChannel *ulCh = new RadioChannel();
  BandwidthManager *spectrum = new BandwidthManager(10, 10, spectrum_index*offset, spectrum_index*offset, true); // 10+10=20 MHz in Dl and Ul, true to abilitate TDD

  spectrum_index++;

  // UE-IAB
  RadioChannel *dlCh_relay = new RadioChannel();
  RadioChannel *ulCh_relay = new RadioChannel();
  BandwidthManager *spectrum_relay = new BandwidthManager(10, 10, spectrum_index*offset, spectrum_index*offset, true); // 10+10=20 MHz in Dl and Ul, true to abilitate TDD

  // SET QoS PARAMETERS
  double maxD = 10; // maxD is the maximum tolerable delay of the transmissions (s)
  QoSParameters *qos = new QoSParameters();
  qos->SetMaxDelay(maxD);

  // APPLICATION PERIODS
  double startTime = 0.1; // s
  double duration = 5;  // s

  // FLOW INFORMATION
  int dstPort = 0;    //Port on the UE
  int srcPort = 100;    //Port on the GNodeB (Donor)
  int dstID = 1;      //It can be assumed the ID as a L3 address, i.e., the IP address
  int srcID = 4;      
  ClassifierParameters *cp = new ClassifierParameters(srcID,
                                                      dstID,
                                                      srcPort,
                                                      dstPort,
                                                      TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);


  // Create Donor GNodeB
  int idGnb = 4;
  int GnbTxAntennas = 1;
  int GnbRxAntennas = 1;
  GNodeB *gnb = networkManager->CreateGnodeb(idGnb, cell, posX + 900, posY, GnbTxAntennas, GnbRxAntennas, dlCh, ulCh, spectrum);
  gnb->SetRandomAccessType(m_GnbRandomAccessType); //Maybe not necessary for downlink case
  gnb->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  gnb->SetULScheduler(GNodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT);

  // Create IAB Node, it is a specialized GNodeB
  int idIab = 2;
  int IabTxAntennas = 1;
  int IabRxAntennas = 1;
  IABNode *iab = networkManager->CreateIABNode(idIab, cell, posX, posY, IabTxAntennas, IabRxAntennas, dlCh_relay, ulCh_relay, spectrum_relay);
  iab->SetRandomAccessType(m_GnbRandomAccessType); //Maybe not necessary for downlink case
  iab->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  iab->SetULScheduler(GNodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT);


  //Speed is required even if the position is constant, it is used to compute the fast fading
  int speed = 3; // km/h

  // Create relay MT
  int idMt = 3;
  int posX_mt = iab->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX(); // MT and DU in the same position
  int posY_mt = iab->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY();
  int MtTxAntennas = 1;
  int MtRxAntennas = 1;
  UserEquipment *mt = networkManager->CreateMobileTermination(idMt, posX_mt, posY_mt, speed, 0, MtTxAntennas, MtRxAntennas, cell, gnb, iab);
  mt->SetRandomAccessType(m_UeRandomAccessType); //Maybe not necessary for downlink case

  // INSTALL APPLICATION
  int applicationID = 0;
  CBR *app = new CBR();
  app->SetApplicationID(applicationID);
  app->SetSource(gnb);
  app->SetDestination(mt);
  app->SetSourcePort(srcPort);
  app->SetDestinationPort(dstPort);
  app->SetInterval(0.5);
  app->SetSize(10);
  app->SetClassifierParameters(cp);
  app->SetQoSParameters(qos);
  app->SetStartTime(startTime);
  app->SetStopTime(startTime + duration);

  // Mount MT on IABNode
  iab->SetMobileTermination(mt);

  // Create UE
  int idUe = 1;
  int posX_ue = posX - 900; // m
  int posY_ue = posY;      // m
  int UeTxAntennas = 1;
  int UeRxAntennas = 1;
  UserEquipment *ue = networkManager->CreateUserEquipment(idUe, posX_ue, posY_ue, speed, 0, UeTxAntennas, UeRxAntennas, cell, iab);
  ue->SetRandomAccessType(m_UeRandomAccessType); //Maybe not necessary for downlink case

  // INSTALL RELAY SENDER IAB->UE
  int relay_applicationID = 1;
  RelaySender *relay_sender = new RelaySender();
  relay_sender->SetApplicationID(relay_applicationID);
  relay_sender->SetSource(iab);
  relay_sender->SetDestination(ue);
  relay_sender->SetSourcePort(srcPort); 
  relay_sender->SetDestinationPort(dstPort); 
  relay_sender->SetClassifierParameters(cp);
  relay_sender->SetQoSParameters(qos);
  relay_sender->SetStartTime(startTime);
  relay_sender->SetStopTime(startTime + duration);

  // RUN SIMULATION
  simulator->SetStop(startTime + duration + 0.1);
  simulator->Run();
}
