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
 * Author: Telematics Lab <telematics-dev@poliba.it>
 */


#include "application-sink.h"
#include "../../device/IPClassifier/ClassifierParameters.h"
#include "../../componentManagers/NetworkManager.h"
#include "../../core/eventScheduler/simulator.h"
#include "../../load-parameters.h"
#include "../../device/UserEquipment.h"

ApplicationSink::ApplicationSink()
{
  m_classifierParameters = nullptr;
  m_radioBearer = nullptr;
  m_sourceApplication = nullptr;
  m_lastID=-1;
}

void
ApplicationSink::SetClassifierParameters (ClassifierParameters* cp)
{
  m_classifierParameters = cp;
}

ClassifierParameters*
ApplicationSink::GetClassifierParameters (void)
{
  return m_classifierParameters;
}


void
ApplicationSink::SetRadioBearerSink (RadioBearerSink* r)
{
  m_radioBearer = r;
}

RadioBearerSink*
ApplicationSink::GetRadioBearerSink (void)
{
  return m_radioBearer;
}

void
ApplicationSink::SetSourceApplication (Application* a)
{
  m_sourceApplication = a;
}

Application*
ApplicationSink::GetSourceApplication (void)
{
  return m_sourceApplication;
}

void 
ApplicationSink::SetLastID (int id)
{
 m_lastID=id;
}

int 
ApplicationSink::GetLastID (void)
{
  return m_lastID;
}

void
ApplicationSink::Receive (Packet* p)
{
  
  if(p->GetID()<=GetLastID())
  return;
  
  SetLastID(p->GetID());
   
  if (!_APP_TRACING_) return;
  /*
   * Trace format:
   *
   * NODE_ID RX   APPLICATION_TYPE   BEARER_ID  SIZE   SRC_ID   DST_ID   TIME
   */
  cout << "NODE " << m_sourceApplication->GetDestination()->GetIDNetworkNode();
  cout << " RX";

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
       << " B " << m_sourceApplication->GetApplicationID ()
       << " SIZE " <<  p->GetSize() - 12 /*p->GetPacketTags ()->GetApplicationSize ()*/
       << " SRC " << p->GetSourceID ()
       << " DST " << p->GetDestinationID ()
       << " T " << Simulator::Init()->Now()
       << " D " << delay;

  if (GetSourceApplication ()->GetDestination ()->GetNodeType() == NetworkNode::TYPE_UE)
    {
      UserEquipment* ue = (UserEquipment*) GetSourceApplication ()->GetDestination ();
      cout << " " << ue->IsIndoor ();
    }

  cout << endl;


  delete p;
}
