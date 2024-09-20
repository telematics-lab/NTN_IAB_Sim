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

#include "../channel/RadioChannel.h"
#include "../phy/gnb-phy.h"
#include "../phy/ue-phy.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../networkTopology/Cell.h"
#include "../protocolStack/packet/packet-burst.h"
#include "../protocolStack/packet/Packet.h"
#include "../core/eventScheduler/simulator.h"
#include "../flows/application/InfiniteBuffer.h"
#include "../flows/application/VoIP.h"
#include "../flows/application/CBR.h"
#include "../flows/application/TraceBased.h"
#include "../device/IPClassifier/ClassifierParameters.h"
#include "../flows/QoS/QoSParameters.h"
#include "../flows/QoS/QoSForEXP.h"
#include "../flows/QoS/QoSForFLS.h"
#include "../flows/QoS/QoSForM_LWDF.h"
#include "../componentManagers/FrameManager.h"
#include "../utility/RandomVariable.h"
#include "../channel/propagation-model/channel-realization.h"
#include "../phy/wideband-cqi-eesm-error-model.h"
#include "../phy/nbiot-simple-error-model.h"
#include "../phy/BLERTrace/BLERvsSINR_NBIoT_SAT.h"
#include "../load-parameters.h"

#include "../device/UserEquipment.h"
#include "../device/NetworkNode.h"
#include "../device/NetworkNode.h"
#include "../device/GNodeB.h"
#include "../device/CqiManager/cqi-manager.h"
#include "../device/CqiManager/fullband-cqi-manager.h"
#include "../device/CqiManager/wideband-cqi-manager.h"
#include "../channel/propagation-model/propagation-loss-model.h"

#include "../protocolStack/mac/random-access/ue-random-access.h"
#include "../protocolStack/mac/random-access/gnb-random-access.h"
#include "../protocolStack/mac/random-access/enb-nb-iot-random-access.h"
#include "../protocolStack/mac/random-access/ue-nb-iot-random-access.h"
#include "../protocolStack/mac/nb-AMCModule.h"

#include <iostream>
#include <queue>
#include <fstream>
#include <stdlib.h>
#include <cstring>
#include <math.h>

static void nbCell_Satellite_ESA(int argc, char *argv[])
{
    // ./5G-air-simulator nbCell-Sat-ESA 3 1000 154 1 3 1 240 2048 15 1 0 1

    
    
    int nSatellitePerOrbit = atoi(argv[2]); // number of satellites per orbit
    int nbUE = atoi(argv[3]);
    double SNRt = atof(argv[4]); //MCL
    int carriers = atoi(argv[5]);  // 1 carrier -> only 48 preamble
    int MCS = atoi(argv[6]);
    int NRep = atoi(argv[7]);
    int RAOPeriod = atoi(argv[8]);
    int backOffRACH = atoi(argv[9]);
    double spacing = atof(argv[10]); // 15 kHz or 3.75 kHz
    bool edtEnable = false;
    bool harqEnable = atoi(argv[11]);
    bool clustering = atoi(argv[12]); // 1 for clustering, 0 for uniform

    // parametrizzare EDT ed HARQ

    
    int seed;
    if (argc==14)
        seed = atoi(argv[13]);
    else
        seed = -1;
    
    // CONFIGURE SEED
    if (seed >= 0)
        srand (seed);
    else
        srand (time(NULL));
    
    std::mt19937 gen(seed>=0 ? seed : time(NULL));
    extern std::mt19937 commonGen;
    commonGen.seed(seed>=0 ? seed : time(NULL));
    
    cout << "Simulation with SEED = " << seed << endl;

    
    int schedUL = 1; // RR
    //double radius = 0.309; // [km]
    double radius = 320; // [km]
   


   // if(!clustering){
   // 	radius = 0.309; // 30 hectars
   // }

    double bandwidth = 30; // [MHz] max 15MHz
    int tones = 1; // 1,3,12
    int NRU = 5;
    int totPreambleTx = 10; // tentativi di RACH procedure
    int nbCE = 1; // numero coverage class
    
    
    std::map<double, int> ceProb;
    std::map<int, int> maxPreambleTx;
    std::map<int, int> preambleRep;
    std::map<int, int> rarWindow;
    std::map<int, int> nbPre;
    std::map<int, int> rachPeriod;
    std::map<int, int> rachOffset;
    std::map<int, int> boWindow;
    
    
    
    int pp; // ra-ResponseWindowSize = {2, 3, 4, 5, 6, 7, 8, 10}  PDCCH PERIODS
    switch ((int)floor(MCS/4)) {
        case 0:
            pp = 8;
            break;
        case 1:
            pp = 6;
            break;
        case 2:
            pp=4;
            break;
            
        case 3:
            pp=3;
            break;
        default:
            cout << "error :  unsupported MCS configuration" << endl;
            exit(1);
    }
    int G = spacing==15 ? 8 : 32;
    
    // al varariare della coverage class
    for (int i=0; i<nbCE; i++) {
        ceProb.insert(std::make_pair(1, i));
        maxPreambleTx.insert(std::make_pair(i, 10));
        preambleRep.insert(std::make_pair(i, NRep));
        rarWindow.insert(std::make_pair(i, pp * G * NRep));
        nbPre.insert(std::make_pair(i, 48 * carriers)); // nP
        rachPeriod.insert(std::make_pair(i,RAOPeriod ));
        rachOffset.insert(std::make_pair(i, 16)); // dall inizio della visibilita del sat fino alla fine del cell search??
        boWindow.insert(std::make_pair(i, backOffRACH)); // backoff window larga
    }

    double carrierFreq = 2000; //MHz
    double antennaHeight = 500000; // m
    double txPower = 33; // dBm
    double BSNoiseFigure = 3; // dB
    
    // define channel model
    ChannelRealization::ChannelModel model= ChannelRealization::CHANNEL_MODEL_SATELLITE;
    
    
    UeRandomAccess::RandomAccessType m_UeRandomAccessType = UeRandomAccess::RA_TYPE_NB_IOT;
    GnbRandomAccess::RandomAccessType m_GnbRandomAccessType = GnbRandomAccess::RA_TYPE_NB_IOT;
    
    // CREATE COMPONENT MANAGER
    Simulator *simulator = Simulator::Init();
    FrameManager *frameManager = FrameManager::Init();
    NetworkManager* networkManager = NetworkManager::Init();
    
    
    // SET FRAME STRUCTURE
    frameManager->SetFrameStructure(FrameManager::FRAME_STRUCTURE_FDD);
    frameManager->SetNRep(NRep);
    frameManager->SetEDT(edtEnable); // EDT
    frameManager->SetHARQ(harqEnable); // HARQ
    frameManager->SetMCSNBIoTSat(MCS);
    frameManager->SetNRUNBIoTSat(NRU);
    
    //Define Application Container
    CBR CBRApplication[nbUE];
    int cbrApplication = 0;
    int destinationPort = 101;
    int applicationID = 0;
    

    int class30min = 0;
    int class1hour = 0;
    int class2hour = 0;
    int class1day = 0;
    
    // CREATE CELL
    Cell *cell = new Cell (0, radius, 0.005, 0, 0);
    
    networkManager->GetCellContainer ()->push_back (cell);
    
    // CREATE CHANNELS and propagation loss model
    RadioChannel *dlCh = new RadioChannel ();
    RadioChannel *ulCh = new RadioChannel ();
    
    // CREATE SPECTRUM
    BandwidthManager* spectrum = new BandwidthManager (bandwidth, bandwidth, 0, 0);
    spectrum->EnableSatelliteScenario();
    spectrum->CreateNbIoTspectrum(carriers, spacing, tones);
    
    cout << "TTI Length: ";
    frameManager->setTTILength(tones, spacing);
    cout << frameManager->getTTILength() << "ms " << endl;
    
    DEBUG_LOG_START_1(SIM_ENV_SCHEDULER_DEBUG_NB)
    spectrum->Print();
    DEBUG_LOG_END
    
    GNodeB::ULSchedulerType uplink_scheduler_type;
    switch (schedUL)
    {
            
        case 0:
            uplink_scheduler_type = GNodeB::ULScheduler_TYPE_NB_IOT_FIFO;
            cout << "Scheduler NB FIFO "<< endl;
            break;
            
        case 1:
            uplink_scheduler_type = GNodeB::ULScheduler_TYPE_NB_IOT_ROUNDROBIN;
            cout << "Scheduler NB RR "<< endl;
            break;
            
        default:
            uplink_scheduler_type = GNodeB::ULScheduler_TYPE_NB_IOT_FIFO;
            cout << "Scheduler NB FIFO "<< endl;
            break;
    }
    
    // The simulator handles only UPLINK case
    
    //Create GNodeB
    NBIoTSimpleErrorModel *errorModel = new NBIoTSimpleErrorModel();
    GNodeB* gnb = new GNodeB (1, cell, -300310, 0, 500000, nSatellitePerOrbit, true); // scenario satellitare a 500km
    gnb->SetRandomAccessType(m_GnbRandomAccessType);
    gnb->GetPhy ()->SetDlChannel (dlCh);
    gnb->GetPhy ()->SetUlChannel (ulCh);
    gnb->GetPhy ()->SetNoiseFigure(BSNoiseFigure);
    gnb->GetPhy ()->SetCarrierFrequency(carrierFreq);
    gnb->GetPhy ()->SetBandwidthManager (spectrum);
    gnb->GetPhy ()->SetHeight(antennaHeight);
    gnb->GetPhy ()->SetTxPower(txPower);
    //gnb->GetPhy ()->SetmaxSatelliteRange(GetMinDistance4CellSelection()); // in base all'angolo di visibilità scelto con SNR > 0
    //((SatelliteMovement*)gnb->GetMobilityModel())->SetAntennaType(SatelliteMovement::PARABOLIC_REFLECTOR);
    //((SatelliteMovement*)gnb->GetMobilityModel())->SetAntennaType(SatelliteMovement::PATCH_ANTENNA);
    ((SatelliteMovement*)gnb->GetMobilityModel())->SetFixedAreaRadius(radius * 1000); // meters
    ((SatelliteMovement*)gnb->GetMobilityModel())->SetMCLthreshold(SNRt);
    
    //gnb->GetPhy ()->SetmaxSatelliteRange(510000);
    gnb->GetPhy ()->SetErrorModel (errorModel);
    ulCh->AddDevice (gnb);
    gnb->SetULScheduler(uplink_scheduler_type);
    
    
    networkManager->GetGNodeBContainer ()->push_back (gnb);
    cout << "Created gNB - id 1 - Out of visibility"<< endl;
    
    GnbNbIoTRandomAccess* gnbRam = (GnbNbIoTRandomAccess*) gnb->GetMacEntity()->GetRandomAccessManager();
    
    gnbRam->SetMaxPreambleTx(maxPreambleTx);
    gnbRam->SetPreambleRep(preambleRep);
    gnbRam->SetRarWindow(rarWindow);
    gnbRam->SetSubcarriers(nbPre);
    gnbRam->SetResPeriodicity(rachPeriod);
    gnbRam->SetTimeOffset(rachOffset);
    gnbRam->SetBackoff(boWindow);
    
    
    // define simulation times
    
    double nPeriods = 10;
    double duration = ((SatelliteMovement*)gnb->GetMobilityModel())->GetVisibilityPeriod() * nPeriods; // 21600 s = 6 h  // 86400 s = 1 day
    duration = duration + 3600 - ( fmod(duration, 3600)); //round to a complete hour
    duration = 172800;
    double flow_duration = duration;
    
    cout << "Duration: " << duration << " flow: " << flow_duration << endl;

    
    //Create UEs
    int idUE = 2;
    double speedDirection = 0;

    int i_cluster = 0;
    int nCluster = 10;
    double uePerCluster = nbUE/nCluster;

    if(nbUE%3000 == 0){
    	uePerCluster = 3000;
    	nCluster = nbUE / uePerCluster;
    }

    cout <<"Number of cluster:"<< nCluster << endl;

    //double deviceDensity = 100.0 / 1000000.0; // ue/km^2
    double deviceDensity = 9710.0 / 1000000.0; // ue/km^2
    double clusterArea = uePerCluster / deviceDensity;
    double clusterRadius = sqrt(clusterArea / M_PI);
    double cluster_x_pos[nCluster];
    double cluster_y_pos[nCluster];
    double r, theta;
    if (clustering) {
        for (int i = 0; i < nCluster; i++) {
            r = sqrt(((double)rand()/RAND_MAX));
            theta = ((double)rand()/RAND_MAX) * 2 * M_PI;
            cluster_x_pos[i] = 0 + (r * radius*1000 * cos(theta));
            cluster_y_pos[i] = 0 + (r * radius*1000 * sin(theta));
        }
    }
    
    double posX, posY;
    
    for (int i = 0; i < nbUE; i++)
    {
        r = sqrt(((double)rand()/RAND_MAX));
        theta = ((double)rand()/RAND_MAX) * 2 * M_PI;

        if (clustering) {
            //ue's random position - cluster
            posX = cluster_x_pos[i_cluster] + (r * clusterRadius * cos(theta));
            posY = cluster_y_pos[i_cluster] + (r * clusterRadius * sin(theta));
            i_cluster = fmod(++i_cluster, nCluster);
        }
        else {
            //ue's random position - uniform
            posX = 0 + (r * radius*1000 * cos(theta));
            posY = 0 + (r * radius*1000 * sin(theta));
        }
        
        UserEquipment* ue = new UserEquipment (idUE,
                                               posX, posY, 0, speedDirection,
                                               cell,
                                               gnb,
                                               0, //handover false!
                                               Mobility::UE_SATELLITE);
        
        cout << "Created UE - id " << idUE << " position " << posX << " " << posY << endl;
        
        ue->SetRandomAccessType(m_UeRandomAccessType);
        
        ue->SetTimePositionUpdate (0.05); // trigger per la mobilità
        
        ue->GetPhy ()->SetDlChannel (dlCh);
        ue->GetPhy ()->SetUlChannel (ulCh);
        
        
        UeMacEntity* ueMac = (UeMacEntity*) ue->GetProtocolStack()->GetMacEntity();
        UeNbIoTRandomAccess* ueRam =(UeNbIoTRandomAccess*) ueMac->GetRandomAccessManager();
        ueRam->SetCEClassStatic(0);
        ueRam->SetCEClassDynamic(0);
        ueRam->SetMaxFailedAttempts(totPreambleTx);
        
        networkManager->GetUserEquipmentContainer ()->push_back (ue);
        
        // register ue to the gnb
        gnb->RegisterUserEquipment (ue);
        gnb->UpdateAttachedUEs(1);
        
        
        // ERROR model
        ue->GetPhy ()->SetErrorModel (errorModel);
        ChannelRealization* c_ul = new ChannelRealization (ue, gnb, model);
        //c_ul->disableFastFading();
        gnb->GetPhy ()->GetUlChannel ()->GetPropagationLossModel ()->AddChannelRealization (c_ul);
        
                
        //PERIODIC INTER-ARRIVAL TIME SPLIT
        //  5% 	-> 30 minutes 	-> 1800 sec
        // 15% 	-> 60 minutes 	-> 3600 sec
        // 40% 	-> 120 minutes 	-> 7200 sec
        // 40% 	-> 1 day 		-> 86400 sec
        
        std::uniform_int_distribution<> rndCBR (0, 100);
        double randomNumber = rndCBR(gen) / 100.;
        double _cbrInterval = 99999.0;
        
        if(randomNumber <= 0.05){
            _cbrInterval = 1800.0;
            class30min++;
        }else if(randomNumber > 0.05 && randomNumber <= 0.20){
            _cbrInterval = 3600.0;
            class1hour++;
        }else if(randomNumber > 0.20 && randomNumber <= 0.60){
            _cbrInterval = 7200.0;
            class2hour++;
        }else if(randomNumber > 0.60){
            _cbrInterval = 86400.0;
            class1day++;
        }

        _cbrInterval = 21600;
        //_cbrInterval = 144;
        cout << "CBR interval is " << _cbrInterval << endl;
        
        //CREATE UPLINK APPLICATION FOR THIS UE
        std::uniform_real_distribution<> timeDis(0.0, (double) _cbrInterval); 
        //std::uniform_real_distribution<> timeDis(0.0, 60); // intervallo all interno del quale l UE trasmette
        double start_time = .001 + timeDis(gen); // 1 ms + un numero da 0 a CBR_interval
        
        // *** cbr application
        // create application
        CBRApplication[cbrApplication].SetSource (ue);
        CBRApplication[cbrApplication].SetDestination (gnb);
        CBRApplication[cbrApplication].SetApplicationID (applicationID);
        CBRApplication[cbrApplication].SetStopTime(flow_duration);
        CBRApplication[cbrApplication].SetStartTime(start_time);
        //CBRApplication[cbrApplication].SetClassicCBR(false);
        CBRApplication[cbrApplication].SetClassicCBR(true);
        CBRApplication[cbrApplication].SetInterval (_cbrInterval);
        CBRApplication[cbrApplication].SetSize (19);

        //-------------------------------------------------------------------
        
        QoSParameters *qosParameters = new QoSParameters ();
        qosParameters->SetMaxDelay (duration);
        CBRApplication[cbrApplication].SetQoSParameters (qosParameters);
        
        
        //create classifier parameters
        ClassifierParameters *cp = new ClassifierParameters (ue->GetIDNetworkNode(),
                                                             gnb->GetIDNetworkNode(),
                                                             0,
                                                             destinationPort,
                                                             TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
        CBRApplication[cbrApplication].SetClassifierParameters (cp);
        
        cout << "CREATED CBR APPLICATION, ID " << applicationID << endl;
        
        //update counter
        applicationID++;
        cbrApplication++;
        idUE++;
    }
    
    simulator->SetStop(duration);
    simulator->Run ();
    
    
}
