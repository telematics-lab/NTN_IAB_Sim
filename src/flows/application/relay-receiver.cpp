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


#include "relay-receiver.h"
#include "../../device/IPClassifier/ClassifierParameters.h"
#include "../../componentManagers/NetworkManager.h"
#include "../../core/eventScheduler/simulator.h"
#include "../../load-parameters.h"
#include "../../device/UserEquipment.h"
#include "../../device/IABNode.h"
#include "../../flows/application/relay-sender.h"

RelayReceiver::RelayReceiver()
{
  ApplicationSink();
}

void
RelayReceiver::Receive (Packet* p)
{

  if(p->GetID()<=GetLastID())
  return;
  
  SetLastID(p->GetID());

  if (!_APP_TRACING_) return;
  /*
   * Trace format:
   *
   * NODE_ID RX-RELAY   APPLICATION_TYPE   BEARER_ID  SIZE   SRC_ID   DST_ID   TIME
   */

  cout << "NODE " << GetSourceApplication()->GetDestination()->GetIDNetworkNode();
  cout << " RX-RELAY";

  switch (p->GetPacketTags ()->GetApplicationType ())
    {
    case Application::APPLICATION_TYPE_VOIP:
      {
        cout << " VOIP";
        break;
      }
    case Application::APPLICATION_TYPE_TRACE_BASED:
      {
        cout << " VIDEO";
        break;
      }
    case Application::APPLICATION_TYPE_CBR:
      {
        cout << " CBR";
        break;
      }
    case Application::APPLICATION_TYPE_INFINITE_BUFFER:
      {
        cout << " INF_BUF";
        break;
      }
    case Application::APPLICATION_TYPE_FTP2:
      {
        cout << " FTP";
        break;
      }
    default:
      {
        cout << " UNDEFINED";
        break;
      }
    }

  double delay = ((Simulator::Init()->Now() *10000) - (p->GetTimeStamp () *10000)) /10000;
  if (delay < 0.000001) delay = 0.000001;

  cout << " ID " << p->GetID ()
       << " B " << GetSourceApplication()->GetApplicationID ()
       << " SIZE " << p->GetSize() - 12 /*p->GetPacketTags ()->GetApplicationSize ()*/
       << " SRC " << p->GetSourceID ()
       << " DST " << p->GetDestinationID ()
       << " T " << Simulator::Init()->Now()
       << " D " << delay;

  cout << endl;

  if (GetSourceApplication()->GetDestination()->GetNodeType() == NetworkNode::TYPE_RELAY_NODE)
  {
        // Packet received on the DU and needs to be relayed on the MT

        // Retrive the pointer to the actual node
        IABNode *destIAB = dynamic_cast<IABNode *>(GetSourceApplication()->GetDestination());

        // Retrive the pointer to the MT sender application
        RelaySender *sourceRelayApp = dynamic_cast<RelaySender *>(destIAB->GetMobileTermination()->GetProtocolStack()->GetApplicationEntity()->GetApplicationSources()->at(0));

        // Send the packet to the next hop
        p->AddHeaderSize(-7); // Remove the 7 bytes of the lower layers headers
        sourceRelayApp->Send(p); 
  }
  else
  {
        // Packet received on the MT and needs to be relayed on the DU

        // Retrive the pointer to the actual node
        UserEquipment *destMT = dynamic_cast<UserEquipment *>(GetSourceApplication()->GetDestination());

        // Retrive the pointer to the DU sender application
        RelaySender *sourceRelayApp = dynamic_cast<RelaySender *>(destMT->GetIABNode()->GetProtocolStack()->GetApplicationEntity()->GetApplicationSources()->at(0));

        // Send the packet to the next hop
        p->AddHeaderSize(-8); // Remove the 8 bytes of the lower layers headers

        sourceRelayApp->Send(p); 
  }
}
