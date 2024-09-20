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
#include "../mobility/Mobility.h"
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

static void nbCell_Satellite (int argc, char *argv[])
{
	// ./5G-air-simulator nbCell-Sat sat 2 -140.0 1 86400 0.3 3000 30 1 15 1 3 5 4 21600 19 15 1 1 10 4 8 48 128 12 1024 10
						        //nbCell-Sat sat 3 -141.0 1 43200 0.309 3000 30 1 15 1 3 5 4 14400 19 15 1 1 10 4 8 48 240 12 1024 10

    int nbOfZones; // divisione in livelli di MCS visto che non e' presente un feedback del CQI


    CBRApplication[cbrApplication].SetStartTime(start_time);
    CBRApplication[cbrApplication].SetStopTime(flow_duration);
    CBRApplication[cbrApplication].SetInterval ((double) CBR_interval);
    CBRApplication[cbrApplication].SetSize (CBR_size);
    QoSParameters *qosParameters = new QoSParameters ();
    //qosParameters->SetMaxDelay (flow_duration);
    qosParameters->SetMaxDelay (duration);
    CBRApplication[cbrApplication].SetQoSParameters (qosParameters);


    //create classifier parameters
    ClassifierParameters *cp = new ClassifierParameters (ue->GetIDNetworkNode(),
                                                   gnb->GetIDNetworkNode(),
                                                   0,
                                                   destinationPort,
                                                   TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApplication[cbrApplication].SetClassifierParameters (cp);


    simulator->SetStop(duration);
    simulator->Run ();

        
}
