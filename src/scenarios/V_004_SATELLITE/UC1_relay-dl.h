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

#include "../../channel/RadioChannel.h"
#include "../../core/spectrum/bandwidth-manager.h"
#include "../../networkTopology/Cell.h"
#include "../../core/eventScheduler/simulator.h"
#include "../../flows/application/InfiniteBuffer.h"
#include "../../flows/QoS/QoSParameters.h"
#include "../../componentManagers/FrameManager.h"
#include "../../componentManagers/FlowsManager.h"
#include "../../device/GNodeB.h"
#include "../../device/IABNode.h"
#include "../../phy/gnb-phy.h"
#include "../../phy/ue-phy.h"
#include "../../phy/wideband-cqi-eesm-error-model.h"
#include "../../phy/sat-simple-error-model.h"
#include "../../protocolStack/packet/packet-burst.h"
#include "../../protocolStack/packet/Packet.h"
#include "../../device/NetworkNode.h"
#include "../../device/UserEquipment.h"
#include "../../device/Gateway.h"
#include "../../load-parameters.h"
#include "../../device/CqiManager/fullband-cqi-manager.h"
#include "../../device/CqiManager/wideband-cqi-manager.h"
#include "../../channel/propagation-model/channel-realization.h"
#include "../../channel/propagation-model/propagation-loss-model.h"
#include "../../utility/CellPosition.h"
#include "../../flows/application/FTP2.h"
#include "../../flows/application/relay-sender.h"

#include <random>

/*
  Description: inerithed from UC1RelayDl 
*/

//Basic configuration: ./5G-air-simulator V_004_SATELLITE_1 1 1 2 1 2 5000000 200.0 294.5 

static void V_004_SATELLITE_1(int argc, char *argv[])
{
  /*SIMULATION PARAMETERS:*/
  /*1- Number of satellites per orbit (the inter-satellite time is set accordingly)*/
  /*2- Number of UEs in the cell*/
  /*3- Antenna configuration, 1 for SAT1 and 2 for SAT2*/
  /*4- Terrestrial antenna pointing, 0 for false and 1 for true (Any impact in the case of Hendheld-Satellite communication)*/
  /*5- TDD configuration, 0 for TDD1, 1 for TDD2 and 2 for TDD3*/
  /*4- FTP starting payload size*/
  /*5- FTP average interval time*/
  /*8- Simulation Time*/

  /*1*/ int nSatellitePerOrbit = atoi(argv[2]); //1
  /*2*/ int nUEs=atoi(argv[3]); //1
  /*3*/ int SatAntenna = atoi(argv[4]);  //2
  /*4*/ int Pointing = atoi(argv[5]);  //0
  /*5*/ int TDDConfig = atoi(argv[6]);  //0
  /*6*/ int FTPSize = atoi(argv[7]);  //1500
  /*7*/ double FTPInterval = atof(argv[8]);  //0.004
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

  // CREATE CELL 1 (For IAB-node to UE and Satellite IAB-node to IAB-node)
  int idCell = 0;
  int radius = 1;           // km
  int minDistance = 0.0035; // km
  int posX = 0;
  int posY = 0;
  Cell *cell1 = networkManager->CreateCell(idCell, radius, minDistance, posX, posY);

  // CREATE CELL 2 (For gNB to Satellite IAB-node)
  idCell = 1;
  radius = 1;           // km
  minDistance = 0.0035; // km
  posX = 0;
  posY = 0;
  Cell *cell2 = networkManager->CreateCell(idCell, radius, minDistance, posX, posY);

  // CREATE CHANNELS AND SPECTRUM

  // The two different link must use different frequencies
  int spectrum_index = 0;
  int offset = 100; // Note: The logic is derived from RunFrequencyReuseTechniques method

  // gNB - MT
  RadioChannel *dlCh_3 = new RadioChannel();
  RadioChannel *ulCh_3 = new RadioChannel();
  BandwidthManager *spectrum_3 = new BandwidthManager(bandwidth, bandwidth, spectrum_index * offset, spectrum_index * offset, true, spacing); // 20 MHz in Dl and Ul, true to abilitate TDD
  spectrum_3->EnableSatelliteScenario();

  spectrum_index++;

  // Sat IAB - IAB
  RadioChannel *dlCh_2 = new RadioChannel();
  RadioChannel *ulCh_2 = new RadioChannel();
  BandwidthManager *spectrum_2 = new BandwidthManager(bandwidth, bandwidth, spectrum_index * offset, spectrum_index * offset, true, spacing); // 20 MHz in Dl and Ul, true to abilitate TDD
  spectrum_2->EnableSatelliteScenario();

  spectrum_index++;

  // IAB - UE
  RadioChannel *dlCh_1 = new RadioChannel();
  RadioChannel *ulCh_1 = new RadioChannel();
  BandwidthManager *spectrum_1 = new BandwidthManager(bandwidth, bandwidth, spectrum_index * offset, spectrum_index * offset, true); // 20 MHz in Dl and Ul, true to abilitate TDD

  // SET QoS PARAMETERS
  double maxD = duration; // maxD is the maximum tolerable delay of the transmissions (s)
  QoSParameters *qos = new QoSParameters();
  qos->SetMaxDelay(maxD);

  // gNB to IAB Sat FLOW INFORMATION
  int srcPort = 0;   // Port on the GNodeB (Donor)
  int dstPort = 100; // Port on the IAB Sat MT
  int srcID = 6;     // It can be assumed the ID as a L3 address, i.e., the IP address
  int dstID = 5;
  ClassifierParameters *cp1 = new ClassifierParameters(srcID,
                                                      dstID,
                                                      srcPort,
                                                      dstPort,
                                                      TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
  // IAB Sat to IAB FLOW INFORMATION
  srcPort = 0;   // Port on the IAB Sat DU
  dstPort = 100; // Port on the IAB
  srcID = 4;     // It can be assumed the ID as a L3 address, i.e., the IP address
  dstID = 3;
  ClassifierParameters *cp2 = new ClassifierParameters(srcID,
                                                      dstID,
                                                      srcPort,
                                                      dstPort,
                                                      TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);

  // Speed is required even if the position is constant, it is used to compute the fast fading
  int speed = 3; // km/h

  // Create terrestrial GNodeB
  int idGnb = 6;
  int GnbTxAntennas = 1;
  int GnbRxAntennas = 1;
  GNodeB *gnb = networkManager->CreateGnodeb(idGnb, cell2, 0, 0, GnbTxAntennas, GnbRxAntennas, dlCh_3, ulCh_3, spectrum_3);
  gnb->SetRandomAccessType(m_GnbRandomAccessType);
  gnb->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  gnb->SetULScheduler(GNodeB::ULScheduler_TYPE_ROUNDROBIN);
  gnb->GetPhy()->SetCarrierFrequency(carrierFreq); // Check
  gnb->GetPhy()->SetErrorModel(errorModel);

  // Create IAB Node Sat
  int idSatIab = 4;
  int SatIabTxAntennas = 1;
  int SatIabRxAntennas = 1;
  IABNode *iab_sat = networkManager->CreateSatIABNode(isLEO, idSatIab, cell1, posX - 1039230, posY, 600000, SatIabTxAntennas, SatIabRxAntennas, dlCh_2, ulCh_2, spectrum_2, nSatellitePerOrbit);
  iab_sat->SetRandomAccessType(m_GnbRandomAccessType);
  iab_sat->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  iab_sat->SetULScheduler(GNodeB::ULScheduler_TYPE_ROUNDROBIN);
  iab_sat->GetPhy()->SetCarrierFrequency(carrierFreq);
  iab_sat->GetPhy()->SetErrorModel(errorModel);
  ((SatelliteMovement *)iab_sat->GetMobilityModel())->SetFixedAreaRadius(radius * 200);
  if(SatAntenna == 1)
  ((SatelliteMovement *)iab_sat->GetMobilityModel())->SetAntennaType(SatelliteMovement::SAT1);
  else
  ((SatelliteMovement *)iab_sat->GetMobilityModel())->SetAntennaType(SatelliteMovement::SAT2);
  if(Pointing == 0)
  ((SatelliteMovement *)iab_sat->GetMobilityModel())->SetPointing(false);
  else
  ((SatelliteMovement *)iab_sat->GetMobilityModel())->SetPointing(true);

  // Create relay MT Sat
  int idSatMt = 5;
  int posX_mt_sat = iab_sat->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX(); // MT and DU in the same position
  int posY_mt_sat = iab_sat->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY();
  int SatMtTxAntennas = 1;
  int SatMtRxAntennas = 1;
  UserEquipment *mt_sat = networkManager->CreateSatMobileTermination(idSatMt, posX_mt_sat, posY_mt_sat, speed, 0, SatMtTxAntennas, SatMtRxAntennas, cell2, gnb, iab_sat);
  mt_sat->SetRandomAccessType(m_UeRandomAccessType);
  mt_sat->GetPhy()->SetErrorModel(errorModel);

  // INSTALL APPLICATION gNB->Sat IAB
  int applicationID = 0;
  FTP2 *app = new FTP2();
  app->SetApplicationID(applicationID);
  app->SetSource(gnb);
  app->SetDestination(mt_sat);
  app->SetSourcePort(srcPort);
  app->SetDestinationPort(dstPort);
  app->SetAverageInterval(FTPInterval);
  app->SetSize(FTPSize);
  app->SetClassifierParameters(cp1);
  app->SetQoSParameters(qos);
  app->SetStartTime(startTime);
  app->SetStopTime(startTime + duration);

  // Create IAB Node
  int idIab = 2;
  int IabTxAntennas = 1;
  int IabRxAntennas = 1;
  IABNode *iab = networkManager->CreateIABNode(idIab, cell1, 0, 0, IabTxAntennas, IabRxAntennas, dlCh_1, ulCh_1, spectrum_1);
  iab->SetRandomAccessType(m_GnbRandomAccessType);
  iab->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  iab->SetULScheduler(GNodeB::ULScheduler_TYPE_ROUNDROBIN);

  // Create relay MT
  int idMt = 3;
  int posX_mt = iab->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX(); // MT and DU in the same position
  int posY_mt = iab->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY();
  int MtTxAntennas = 1;
  int MtRxAntennas = 1;
  UserEquipment *mt = networkManager->CreateSatMobileTermination(idMt, 0, 0, speed, 0, MtTxAntennas, MtRxAntennas, cell1, iab_sat, iab);
  mt->SetRandomAccessType(m_UeRandomAccessType);
  mt->GetPhy()->SetErrorModel(errorModel);

 // INSTALL RELAY SENDER Sat IAB->MT
 int relay_applicationID_1 = 1;
 RelaySender *relay_sender_1 = new RelaySender();
 relay_sender_1->SetApplicationID(relay_applicationID_1);
 relay_sender_1->SetSource(iab_sat);
 relay_sender_1->SetDestination(mt);
 relay_sender_1->SetSourcePort(srcPort);
 relay_sender_1->SetDestinationPort(dstPort);
 relay_sender_1->SetClassifierParameters(cp2);
 relay_sender_1->SetQoSParameters(qos);
 relay_sender_1->SetStartTime(startTime);
 relay_sender_1->SetStopTime(startTime + duration);

 // Mount MT on IABNode
 iab->SetMobileTermination(mt);

 // Mount MT Sat on IABNode Sat
 iab_sat->SetMobileTermination(mt_sat);

vector<UserEquipment*> UEs;
vector<RelaySender*> APPs;

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
  UEs.push_back(networkManager->CreateUserEquipment(idUe, posX_ue, posY_ue, speed, 0, UeTxAntennas, UeRxAntennas, cell1, iab));
  UEs.back()->SetRandomAccessType(m_UeRandomAccessType);

  UEs.back()->GetPhy()->SetNoiseFigure(7);
  UEs.back()->GetPhy()->SetCarrierFrequency(2000);
  UEs.back()->GetPhy()->SetTxPower(35); // 23
  UEs.back()->GetPhy()->GetAntennaParameters()->SetType(Phy::AntennaParameters::ANTENNA_TYPE_OMNIDIRECTIONAL);

  // IAB to Sat FLOW INFORMATION
  int srcPort = 0;         // Port on the UE
  int dstPort = 100;       // Port on the IAB Node
  int srcID = 2; // It can be assumed the ID as a L3 address, i.e., the IP address
  int dstID = user + 1000;
  ClassifierParameters *cp = new ClassifierParameters(srcID,
                                                      dstID,
                                                      srcPort,
                                                      dstPort,
                                                      TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);

 // INSTALL RELAY SENDER IAB->UE
 int relay_applicationID_2 =  user + 2;
 RelaySender *relay_sender_2 = new RelaySender();
 relay_sender_2->SetApplicationID(relay_applicationID_2);
 relay_sender_2->SetSource(iab);
 relay_sender_2->SetDestination(UEs.back());
 relay_sender_2->SetSourcePort(srcPort);
 relay_sender_2->SetDestinationPort(dstPort);
 relay_sender_2->SetClassifierParameters(cp);
 relay_sender_2->SetQoSParameters(qos);
 relay_sender_2->SetStartTime(startTime + delayIABProcedures);
 relay_sender_2->SetStopTime(startTime + duration + delayIABProcedures);
 APPs.push_back(relay_sender_2);

}
// RUN SIMULATION
simulator->SetStop(startTime + duration + 0.1);
simulator->Run();
}
