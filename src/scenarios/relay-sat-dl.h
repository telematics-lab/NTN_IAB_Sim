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
#include "../phy/sat-simple-error-model.h"
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

static void RelaySatExampleDl()
{
  /*
  The following Scenario simulate the case in wich a Satellite GNodeB (ID = 4) try to periodically send packets
  through a CBR Application, being in the range of a MobileTermination (ID = 3) in charge to receive packets and relays
  them via an IABNode (ID = 2) to a UE (ID = 1).
  */

  // SIMULATION PERIODS
  double startTime = 0.1; // s
  double duration = 294.5;    // s
  //
  double bandwidth = 10; // [MHz]
  double carrierFreq = 2000;

  double spacing = 15; // 15 kHz or 30 kHz or 60 kHz
  bool handover = false;
  double BSFeederLoss = 0; // 2

  int nSatellitePerOrbit = 1; // numero di satelliti per orbita
  double CBR_interval = 1;
  int CBR_size = 1500;

  SatSimpleErrorModel *errorModel = new SatSimpleErrorModel();
  // CREATE COMPONENT MANAGERS
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager *networkManager = NetworkManager::Init();
  FlowsManager *flowsManager = FlowsManager::Init();

  // CONFIGURE SEED
  int seed = time(nullptr);
  srand(seed);

  // SET RANDOM ACCESS
  UeRandomAccess::RandomAccessType m_UeRandomAccessType = UeRandomAccess::RA_TYPE_BASELINE;
  GnbRandomAccess::RandomAccessType m_GnbRandomAccessType = GnbRandomAccess::RA_TYPE_BASELINE;

  // SET DUPLEXING MODE
  // frameManager->SetFrameStructure(FrameManager::FRAME_STRUCTURE_FDD); // Default value
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
  int offset = 100; // Note: The logic is derived from RunFrequencyReuseTechniques method

  // MT-gNB
  RadioChannel *dlCh = new RadioChannel();
  RadioChannel *ulCh = new RadioChannel();
  BandwidthManager *spectrum = new BandwidthManager(bandwidth, bandwidth, spectrum_index * offset, spectrum_index * offset, true, spacing); // 20 MHz in Dl and Ul, true to abilitate TDD
  spectrum->EnableSatelliteScenario();

  spectrum_index++;

  // UE-IAB
  RadioChannel *dlCh_relay = new RadioChannel();
  RadioChannel *ulCh_relay = new RadioChannel();
  BandwidthManager *spectrum_relay = new BandwidthManager(bandwidth, bandwidth, spectrum_index * offset, spectrum_index * offset, true); // 20 MHz in Dl and Ul, true to abilitate TDD

  // SET QoS PARAMETERS
  double maxD = duration; // maxD is the maximum tolerable delay of the transmissions (s)
  QoSParameters *qos = new QoSParameters();
  qos->SetMaxDelay(maxD);

  // FLOW INFORMATION
  int dstPort = 0;   // Port on the UE
  int srcPort = 100; // Port on the GNodeB (Donor)
  int dstID = 1;     // It can be assumed the ID as a L3 address, i.e., the IP address
  int srcID = 4;
  ClassifierParameters *cp = new ClassifierParameters(srcID,
                                                      dstID,
                                                      srcPort,
                                                      dstPort,
                                                      TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);

  // Create Satellite Donor GNodeB
  int idGnb = 4;
  int GnbTxAntennas = 1;
  int GnbRxAntennas = 1;
  GNodeB *gnb = networkManager->CreateSatGnodeb( false, idGnb, cell, posX - 1039230, posY, 600000, GnbTxAntennas, GnbRxAntennas, dlCh, ulCh, spectrum, nSatellitePerOrbit);
  gnb->SetRandomAccessType(m_GnbRandomAccessType);
  gnb->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  gnb->SetULScheduler(GNodeB::ULScheduler_TYPE_ROUNDROBIN);
  gnb->GetPhy()->SetCarrierFrequency(carrierFreq); // Check
  gnb->GetPhy()->SetErrorModel(errorModel);
  ((SatelliteMovement *)gnb->GetMobilityModel())->SetFixedAreaRadius(radius * 1000);
  ((SatelliteMovement *)gnb->GetMobilityModel())->SetAntennaType(SatelliteMovement::PATCH_ANTENNA);

  // Create IAB Node, it is a specialized GNodeB
  int idIab = 2;
  int IabTxAntennas = 1;
  int IabRxAntennas = 1;
  IABNode *iab = networkManager->CreateIABNode(idIab, cell, posX, posY, IabTxAntennas, IabRxAntennas, dlCh_relay, ulCh_relay, spectrum_relay);
  iab->SetRandomAccessType(m_GnbRandomAccessType);
  iab->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  iab->SetULScheduler(GNodeB::ULScheduler_TYPE_ROUNDROBIN);

  // Speed is required even if the position is constant, it is used to compute the fast fading
  int speed = 3; // km/h

  // Create relay MT
  int idMt = 3;
  int posX_mt = iab->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX(); // MT and DU in the same position
  int posY_mt = iab->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY();
  int MtTxAntennas = 1;
  int MtRxAntennas = 1;
  UserEquipment *mt = networkManager->CreateSatMobileTermination(idMt, posX_mt, posY_mt, speed, 0, MtTxAntennas, MtRxAntennas, cell, gnb, iab);
  mt->SetRandomAccessType(m_UeRandomAccessType);
  mt->GetPhy()->SetErrorModel(errorModel);

  // INSTALL APPLICATION
  int applicationID = 0;
  CBR *app = new CBR();
  app->SetApplicationID(applicationID);
  app->SetSource(gnb);
  app->SetDestination(mt);
  app->SetSourcePort(srcPort);
  app->SetDestinationPort(dstPort);
  app->SetInterval(CBR_interval);
  app->SetSize(CBR_size);
  app->SetClassifierParameters(cp);
  app->SetQoSParameters(qos);
  app->SetStartTime(startTime);
  app->SetStopTime(startTime + duration);

  // Mount MT on IABNode
  iab->SetMobileTermination(mt);

  // Create UE
  int idUe = 1;
  int posX_ue = posX - 900; // m
  int posY_ue = posY;       // m
  int UeTxAntennas = 1;
  int UeRxAntennas = 1;
  UserEquipment *ue = networkManager->CreateUserEquipment(idUe, posX_ue, posY_ue, speed, 0, UeTxAntennas, UeRxAntennas, cell, iab);
  ue->SetRandomAccessType(m_UeRandomAccessType);

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
