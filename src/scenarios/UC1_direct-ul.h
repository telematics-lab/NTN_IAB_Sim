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

/*
  The following Scenario simulate the case in wich a group of UEs (ID = 1000, 1001, 1002, 1003...) try to periodically send packet
  through a CBR Application, directly to a Satellite IAB Node (ID = 2), in charge to receive packets and relays
  them via a MobileTermination (ID = 3) to a terrestrial GNodeB (ID = 4).
*/

//Basic configuration: ./5G-air-simulator UC1DirectUl 1 1 2 0 0 250 5 294.5 

static void UC1DirectUl(int argc, char *argv[])
{
  /*SIMULATION PARAMETERS:*/
  /*1- Number of satellites per orbit (the inter-satellite time is set accordingly)*/
  /*2- Number of UEs in the cell*/
  /*3- Antenna configuration, 1 for SAT1 and 2 for SAT2*/
  /*4- Terrestrial antenna pointing, 0 for false and 1 for true (Any impact in the case of Hendheld-Satellite communication)*/
  /*5- TDD configuration, 0 for TDD1, 1 for TDD2 and 2 for TDD3*/
  /*6- CBR payload size*/
  /*7- CBR interval size*/
  /*8- Simulation Time*/

  /*1*/ int nSatellitePerOrbit = atoi(argv[2]); //1
  /*2*/ int nUEs=atoi(argv[3]); //1
  /*3*/ int SatAntenna = atoi(argv[4]);  //2
  /*4*/ int Pointing = atoi(argv[5]);  //0
  /*5*/ int TDDConfig = atoi(argv[6]);  //0
  /*6*/ int CBRSize = atoi(argv[7]);  //1500
  /*7*/ double CBRInterval = atof(argv[8]);  //0.004
  /*8*/ double duration = atof(argv[9]);  // [s] //294.5

  /*SIMULATION SETTINGS*/

  //SET SATELLITE PLATFORM

  bool isLEO = true;

  // SET TIME PERIODS
  double startTime = 0.1;          // [s]
  double delayIABProcedures = 0.0; // [s]

  // SET SPECTRUM PARAMETERS
  double bandwidth = 10;     // [MHz]
  double carrierFreq = 2000; // [kHz]
  double spacing = 15;       // [kHz]

  // SET ERROR MODEL
  SatSimpleErrorModel *errorModel = new SatSimpleErrorModel();

  // CREATE COMPONENT MANAGERS
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager *networkManager = NetworkManager::Init();
  FlowsManager *flowsManager = FlowsManager::Init();

  // CONFIGURE SEED
  int seed = 1;
  srand(seed);

  // SET RANDOM ACCESS
  UeRandomAccess::RandomAccessType m_UeRandomAccessType = UeRandomAccess::RA_TYPE_BASELINE;
  GnbRandomAccess::RandomAccessType m_GnbRandomAccessType = GnbRandomAccess::RA_TYPE_BASELINE;

  // SET DUPLEXING MODE
  frameManager->SetFrameStructure(FrameManager::FRAME_STRUCTURE_TDD);
  frameManager->SetTDDFrameConfiguration(TDDConfig + 7);

  // CREATE CELL 1 (For UE to Satellite IAB-node)
  int idCell = 0;
  int radius = 1;           // [km]
  int minDistance = 0.0035; // [km]
  int posX = 0;
  int posY = 0;
  Cell *cell1 = networkManager->CreateCell(idCell, radius, minDistance, posX, posY);

  // CREATE CELL 2 (For Satellite IAB-node to gNB)
   idCell = 1;
   radius = 1;           // [km]
   minDistance = 0.0035; // [km]
   posX = 0;
   posY = 0;
  Cell *cell2 = networkManager->CreateCell(idCell, radius, minDistance, posX, posY);

  // CREATE CHANNELS AND SPECTRUM

  // The two different link must use different frequencies
  int spectrum_index = 0;
  int offset = 100; // Note: The logic is derived from RunFrequencyReuseTechniques method

  // MT - gNB
  RadioChannel *dlCh = new RadioChannel();
  RadioChannel *ulCh = new RadioChannel();
  BandwidthManager *spectrum = new BandwidthManager(bandwidth, bandwidth, spectrum_index * offset, spectrum_index * offset, true, spacing); // 20 MHz in Dl and Ul, true to abilitate TDD
  spectrum->EnableSatelliteScenario();

  spectrum_index++;

  // UE - IAB
  RadioChannel *dlCh_relay = new RadioChannel();
  RadioChannel *ulCh_relay = new RadioChannel();
  BandwidthManager *spectrum_relay = new BandwidthManager(bandwidth, bandwidth, spectrum_index * offset, spectrum_index * offset, true, spacing); // 20 MHz in Dl and Ul, true to abilitate TDD
  spectrum_relay->EnableSatelliteScenario();

  // SET QoS PARAMETERS
  double maxD = duration; // maxD is the maximum tolerable delay of the transmissions (s)
  QoSParameters *qos = new QoSParameters();
  qos->SetMaxDelay(maxD);

  // IAB Sat to gNB FLOW INFORMATION
  int srcPort = 0;   // Port on the MT
  int dstPort = 100; // Port on the GNodeB (Donor)
  int srcID = 3;     // It can be assumed the ID as a L3 address, i.e., the IP address
  int dstID = 4;
  ClassifierParameters *cp = new ClassifierParameters(srcID,
                                                      dstID,
                                                      srcPort,
                                                      dstPort,
                                                      TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);

  // Speed is required even if the position is constant, it is used to compute the fast fading
  int speed = 3; // km/h

  // Create terrestrial GNodeB
  int idGnb = 4;
  int GnbTxAntennas = 1;
  int GnbRxAntennas = 1;
  GNodeB *gnb = networkManager->CreateGnodeb(idGnb, cell2, 0, 0, GnbTxAntennas, GnbRxAntennas, dlCh, ulCh, spectrum);
  gnb->SetRandomAccessType(m_GnbRandomAccessType);
  gnb->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  gnb->SetULScheduler(GNodeB::ULScheduler_TYPE_ROUNDROBIN);
  gnb->GetPhy()->SetErrorModel(errorModel);

  // Create IAB Node Sat
  int idIab = 2;
  int IabTxAntennas = 1;
  int IabRxAntennas = 1;
  IABNode *iab = networkManager->CreateSatIABNode(isLEO, idIab, cell1, posX - 1039230, posY, 600000, IabTxAntennas, IabRxAntennas, dlCh_relay, ulCh_relay, spectrum_relay, nSatellitePerOrbit);
  iab->SetRandomAccessType(m_GnbRandomAccessType);
  iab->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  iab->SetULScheduler(GNodeB::ULScheduler_TYPE_ROUNDROBIN);
  iab->GetPhy()->SetCarrierFrequency(carrierFreq);
  iab->GetPhy()->SetErrorModel(errorModel);
  ((SatelliteMovement *)iab->GetMobilityModel())->SetFixedAreaRadius(radius * 200);
  if(SatAntenna == 1)
  ((SatelliteMovement *)iab->GetMobilityModel())->SetAntennaType(SatelliteMovement::SAT1);
  else
  ((SatelliteMovement *)iab->GetMobilityModel())->SetAntennaType(SatelliteMovement::SAT2);
  if(Pointing == 0)
  ((SatelliteMovement *)iab->GetMobilityModel())->SetPointing(false);
  else
  ((SatelliteMovement *)iab->GetMobilityModel())->SetPointing(true);


  // Create relay MT
  int idMt = 3;
  int posX_mt = iab->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX(); // MT and DU in the same position
  int posY_mt = iab->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY();
  int MtTxAntennas = 1;
  int MtRxAntennas = 1;
  UserEquipment *mt = networkManager->CreateSatMobileTermination(idMt, posX_mt, posY_mt, speed, 0, MtTxAntennas, MtRxAntennas, cell2, gnb, iab);
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
  
  vector<UserEquipment *> UEs;
  vector<CBR *> APPs;

  // Create UEs
  for (int user = 0; user < nUEs; user++)
  {
    int idUe = user + 1000;
    int posX_ue = posX + 100; // m
    int posY_ue = posY + 100; // m
    cout << "User "
         << " at [" << posX_ue << "," << posY_ue << "]" << endl;
    int UeTxAntennas = 1;
    int UeRxAntennas = 1;
    UEs.push_back(networkManager->CreateSatUserEquipment(idUe, posX_ue, posY_ue, speed, 0, UeTxAntennas, UeRxAntennas, cell1, iab));
    UEs.back()->SetRandomAccessType(m_UeRandomAccessType);

    UEs.back()->GetPhy()->SetNoiseFigure(7);
    UEs.back()->GetPhy()->SetCarrierFrequency(2000);
    UEs.back()->GetPhy()->SetTxPower(35); // 23
    UEs.back()->GetPhy()->GetAntennaParameters()->SetType(Phy::AntennaParameters::ANTENNA_TYPE_OMNIDIRECTIONAL);

    // UE to Sat FLOW INFORMATION
    int srcPort = 0;         // Port on the UE
    int dstPort = 100;       // Port on the IAB Sat
    int srcID = user + 1000; // It can be assumed the ID as a L3 address, i.e., the IP address
    int dstID = 2;
    ClassifierParameters *cp = new ClassifierParameters(srcID,
                                                      dstID,
                                                      srcPort,
                                                      dstPort,
                                                      TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);

    // INSTALL APPLICATION
    int applicationID = user + 1000;
    APPs.push_back(new CBR());
    APPs.back()->SetApplicationID(applicationID);
    APPs.back()->SetSource(UEs.back());
    APPs.back()->SetDestination(iab);
    APPs.back()->SetSourcePort(srcPort);
    APPs.back()->SetDestinationPort(dstPort);
    APPs.back()->SetInterval(CBRInterval);
    APPs.back()->SetSize(CBRSize);
    APPs.back()->SetClassifierParameters(cp);
    APPs.back()->SetQoSParameters(qos);
    APPs.back()->SetStartTime(startTime + delayIABProcedures);
    APPs.back()->SetStopTime(startTime + duration + delayIABProcedures);
  }

  // RUN SIMULATION
  simulator->SetStop(startTime + duration + delayIABProcedures);// + 0.1);
  simulator->Run();
}
