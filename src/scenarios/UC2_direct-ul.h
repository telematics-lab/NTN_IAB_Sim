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
  Description: TODO
*/

//Basic configuration: ./5G-air-simulator UC2DirectUl 1 2 45 250 5 60.0

static void UC2DirectUl(int argc, char *argv[])
{
  /*SIMULATION PARAMETERS:*/
  /*1- Number of UEs in the cell*/
  /*2- Antenna configuration, 0 for SAT3 and 1 for SAT4*/
  /*3- Elevation Angle*/
  /*4- CBR payload size*/
  /*5- CBR interval size*/
  /*6- Simulation Time*/

  /*1*/ int nUEs=atoi(argv[2]); //1
  /*2*/ int SatAntenna = atoi(argv[3]);  //2
  /*3*/ int ElAngle = atoi(argv[4]); //45
  /*4*/ int CBRSize = atoi(argv[5]);  //250
  /*5*/ double CBRInterval = atof(argv[6]);  //5
  /*6*/ double duration = atof(argv[7]);  // [s] //60.0

  /*SIMULATION SETTINGS*/

  //SET SATELLITE PLATFORM
  int nSatellitePerOrbit = 1;
  bool isLEO = false;

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
  frameManager->SetTDDFrameConfiguration(11);

  // CREATE CELL 1 (For UE to IAB-node and IAB-node to Satellite gNB)
  int idCell = 0;
  int radius = 1;           // km
  int minDistance = 0.0035; // km
  int posX = 0;
  int posY = 0;
  Cell *cell1 = networkManager->CreateCell(idCell, radius, minDistance, posX, posY);


  // CREATE CHANNELS AND SPECTRUM

  // The two different link must use different frequencies
  int spectrum_index = 0;
  int offset = 100; // Note: The logic is derived from RunFrequencyReuseTechniques method

  // UE - gNB
  RadioChannel *dlCh_2 = new RadioChannel();
  RadioChannel *ulCh_2 = new RadioChannel();
  BandwidthManager *spectrum_2 = new BandwidthManager(bandwidth, bandwidth, spectrum_index * offset, spectrum_index * offset, true, spacing); // 20 MHz in Dl and Ul, true to abilitate TDD
  spectrum_2->EnableSatelliteScenario();

  // SET QoS PARAMETERS
  double maxD = duration; // maxD is the maximum tolerable delay of the transmissions (s)
  QoSParameters *qos = new QoSParameters();
  qos->SetMaxDelay(maxD);

  // Speed is required even if the position is constant, it is used to compute the fast fading
  int speed = 3; // km/h

  // Create gNB Sat
  int idSatGnb = 2;
  int SatGnbTxAntennas = 1;
  int SatGnbRxAntennas = 1;
  GNodeB *gnb_sat = networkManager->CreateSatGnodeb(isLEO, idSatGnb, cell1, 0, 0, 36000, SatGnbTxAntennas, SatGnbRxAntennas, dlCh_2, ulCh_2, spectrum_2, nSatellitePerOrbit);
  gnb_sat->SetRandomAccessType(m_GnbRandomAccessType);
  gnb_sat->SetDLScheduler(GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  gnb_sat->SetULScheduler(GNodeB::ULScheduler_TYPE_ROUNDROBIN);
  gnb_sat->GetPhy()->SetCarrierFrequency(carrierFreq);
  gnb_sat->GetPhy()->SetErrorModel(errorModel);
  ((SatelliteMovement *)gnb_sat->GetMobilityModel())->SetFixedAreaRadius(radius * 200);
  ((SatelliteMovement *)gnb_sat->GetMobilityModel())->SetGEOangle(ElAngle);
  if(SatAntenna == 1)
  ((SatelliteMovement *)gnb_sat->GetMobilityModel())->SetAntennaType(SatelliteMovement::SAT3);
  else
  ((SatelliteMovement *)gnb_sat->GetMobilityModel())->SetAntennaType(SatelliteMovement::SAT4);

vector<UserEquipment*> UEs;
vector<CBR*> APPs;

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
  UEs.push_back(networkManager->CreateSatUserEquipment(idUe, posX_ue, posY_ue, speed, 0, UeTxAntennas, UeRxAntennas, cell1, gnb_sat));
  UEs.back()->SetRandomAccessType(m_UeRandomAccessType);

  UEs.back()->GetPhy()->SetNoiseFigure(7);
  UEs.back()->GetPhy()->SetCarrierFrequency(2000);
  UEs.back()->GetPhy()->SetTxPower(35); // 23
  UEs.back()->GetPhy()->GetAntennaParameters()->SetType(Phy::AntennaParameters::ANTENNA_TYPE_OMNIDIRECTIONAL);

  // IAB to Sat FLOW INFORMATION
  int srcPort = 0;         // Port on the UE
  int dstPort = 100;       // Port on the IAB Node
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
  APPs.back()->SetDestination(gnb_sat);
  APPs.back()->SetSourcePort(srcPort);
  APPs.back()->SetDestinationPort(dstPort);
  APPs.back()->SetInterval(CBRInterval);
  APPs.back()->SetSize(CBRSize);
  APPs.back()->SetClassifierParameters(cp);
  APPs.back()->SetQoSParameters(qos);
  APPs.back()->SetStartTime(startTime);
  APPs.back()->SetStopTime(startTime + duration);
}
// RUN SIMULATION
simulator->SetStop(startTime + duration + 0.1);
simulator->Run();
}