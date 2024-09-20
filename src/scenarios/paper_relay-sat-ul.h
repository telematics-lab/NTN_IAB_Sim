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

#include <random>

static void PaperRelaySatExampleUl(int argc, char *argv[])
{
  /*
  The following Scenario simulate the case in wich a UE (ID = 1) try to periodically send packet
  through a CBR Application, being in the range of a IAB Node (ID = 2) in charge to receive packets and relays
  them via a MobileTermination (ID = 3) to a GNodeB (ID = 4).
  */
  int nSatellitePerOrbit = atoi(argv[2]); // numero di satelliti per orbita
  int nUEs=atoi(argv[3]);
  int sat_antenna = atoi(argv[4]); //1 for patch, 2 for dish reflector
  int tdd_pattern=atoi(argv[5]); //1,2,3,4 

  // SIMULATION PERIODS
  double startTime = 0.1; // s
  double duration = 294.5;    // s 
  //

  double bandwidth = 10; // [MHz]
  double carrierFreq = 2000;

  double spacing = 15; // 15 kHz or 30 kHz or 60 kHz
  bool handover = false;
  double BSFeederLoss = 0; // 2

  
  double CBR_interval = 0.004;
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
  frameManager->SetTDDFrameConfiguration(6+tdd_pattern);

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

  // IAB to Sat FLOW INFORMATION
  int srcPort = 0;   // Port on the MT
  int dstPort = 100; // Port on the GNodeB (Donor)
  int srcID = 3;     // It can be assumed the ID as a L3 address, i.e., the IP address
  int dstID = 4;
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
  if(sat_antenna==1)
  ((SatelliteMovement *)gnb->GetMobilityModel())->SetAntennaType(SatelliteMovement::PATCH_ANTENNA);
  else
  ((SatelliteMovement *)gnb->GetMobilityModel())->SetAntennaType(SatelliteMovement::PARABOLIC_REFLECTOR);

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

  // INSTALL RELAY SENDER MT->GNB
  int relay_applicationID = 1;
  RelaySender *relay_sender = new RelaySender();
  relay_sender->SetApplicationID(relay_applicationID);
  relay_sender->SetSource(mt);
  relay_sender->SetDestination(gnb);
  relay_sender->SetSourcePort(srcPort);
  relay_sender->SetDestinationPort(dstPort);
  relay_sender->SetClassifierParameters(cp);
  relay_sender->SetQoSParameters(qos);
  relay_sender->SetStartTime(startTime);
  relay_sender->SetStopTime(startTime + duration);

  // Mount MT on IABNode
  iab->SetMobileTermination(mt);

vector<UserEquipment*> UEs;
vector<CBR*> APPs;
// Create UEs
for(int user=0; user<nUEs; user++){
  
  int idUe = user+1000;
  int posX_ue = posX + (rand()%1000)-500; // m
  int posY_ue = posY + (rand()%1000)-500; // m
  cout << "User "
       << " at [" << posX_ue << "," << posY_ue << "]" << endl;
  int UeTxAntennas = 1;
  int UeRxAntennas = 1;
  UEs.push_back(networkManager->CreateUserEquipment(idUe, posX_ue, posY_ue, speed, 0, UeTxAntennas, UeRxAntennas, cell, iab));
  UEs.back()->SetRandomAccessType(m_UeRandomAccessType);

  UEs.back()->GetPhy ()->SetNoiseFigure(7);
  UEs.back()->GetPhy ()->SetCarrierFrequency(2000);
  UEs.back()->GetPhy ()->SetTxPower(23); //23
  UEs.back()->GetPhy ()->GetAntennaParameters ()->SetType(Phy::AntennaParameters::ANTENNA_TYPE_OMNIDIRECTIONAL);
           
    // IAB to Sat FLOW INFORMATION
  int srcPort = 0;   // Port on the UE
  int dstPort = 100; // Port on the GNodeB (Donor)
  int srcID = user+1000;     // It can be assumed the ID as a L3 address, i.e., the IP address
  int dstID = 2;
  ClassifierParameters *cp = new ClassifierParameters(srcID,
                                                      dstID,
                                                      srcPort,
                                                      dstPort,
                                                      TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);

  // INSTALL APPLICATION
  int applicationID = user+1000;
  APPs.push_back(new CBR());
  APPs.back()->SetApplicationID(applicationID);
  APPs.back()->SetSource(UEs.back());
  APPs.back()->SetDestination(iab);
  APPs.back()->SetSourcePort(srcPort);
  APPs.back()->SetDestinationPort(dstPort);
  APPs.back()->SetInterval(CBR_interval);
  APPs.back()->SetSize(CBR_size);
  APPs.back()->SetClassifierParameters(cp);
  APPs.back()->SetQoSParameters(qos);
  APPs.back()->SetStartTime(startTime);
  APPs.back()->SetStopTime(startTime + duration);
}
  // RUN SIMULATION
  simulator->SetStop(startTime + duration + 0.1);
  simulator->Run();
}
