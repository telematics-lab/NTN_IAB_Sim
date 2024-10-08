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


#include "VoIP.h"
#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"
#include <math.h>

VoIP::VoIP()
{
  /*
   * G729 codec generates during the ON period a packet with
   * a fixed size (20 bytes). We must add the RTP header (12 bytes)
   */
  m_size = 32; //application + RTP header
  m_stateON = false;
  SetApplicationType (Application::APPLICATION_TYPE_VOIP);
}

void
VoIP::DoStart (void)
{
  Simulator::Init()->Schedule(0.0, &VoIP::Send, this);
}

void
VoIP::DoStop (void)
{}

void
VoIP::ScheduleTransmit (double time)
{
  if ( (Simulator::Init()->Now () + time) < GetStopTime () )
    {
      Simulator::Init()->Schedule(time, &VoIP::Send, this);
    }
}

void
VoIP::Send (void)
{
  if (!m_stateON)
    {
      m_stateON = true;
      //start state ON
      double random = rand() %10000;
      m_stateDuration = -3*log(1-((double)random/10000));
      m_endState = Simulator::Init()->Now () + m_stateDuration;

DEBUG_LOG_START_1(SIM_ENV_APPLICATION_DEBUG)
      cout << " VoIP_DEBUG - Start ON Period, "
                "\n\t Time = " << Simulator::Init()->Now ()
                << "\n\t state ON Duration = " << m_stateDuration
                << "\n\t end ON state = " <<  m_endState << endl;
DEBUG_LOG_END

    }

  //CREATE A NEW PACKET (ADDING UDP, IP and PDCP HEADERS)
  Packet *packet = new Packet ();
  packet->SetID(GetID());
  SetID(GetID()+1);
  packet->SetTimeStamp (Simulator::Init()->Now ());
  packet->SetSize (GetSize ());

  Trace (packet);

  PacketTAGs *tags = new PacketTAGs ();
  tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_VOIP);
  tags->SetApplicationSize (packet->GetSize ());
  packet->SetPacketTags(tags);


  UDPHeader *udp = new UDPHeader (GetClassifierParameters ()->GetSourcePort (),
                                  GetClassifierParameters ()->GetDestinationPort ());
  packet->AddUDPHeader (udp);

  IPHeader *ip = new IPHeader (GetClassifierParameters ()->GetSourceID (),
                               GetClassifierParameters ()->GetDestinationID ());
  packet->AddIPHeader (ip);

  PDCPHeader *pdcp = new PDCPHeader ();
  packet->AddPDCPHeader (pdcp);

  GetRadioBearer()->Enqueue (packet);


  if (Simulator::Init()->Now () <= m_endState)
    {
      ScheduleTransmit (0.02);
    }
  else
    {
      //schedule OFF Period
      m_stateON = false;
      double random = rand() %10000;
      m_stateDuration = -2.23*log(1-((double)random/10000));
      if (m_stateDuration > 6.9)
        {
          m_stateDuration = 6.9;
        }

DEBUG_LOG_START_1(SIM_ENV_APPLICATION_DEBUG)
      cout << " VoIP_DEBUG - Start OFF Period, "
                "\n\t Time = " << Simulator::Init()->Now ()
                << "\n\t state OFF Duration = " << m_stateDuration << endl;
DEBUG_LOG_END

      ScheduleTransmit (m_stateDuration);
    }
}


int
VoIP::GetSize (void) const
{
  return m_size;
}
