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
 * Author: Alessandro Grassi <alessandro.grassi@poliba.it>
 */



#include "FTP2.h"
#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"

#define MAXMTUSIZE 1490

FTP2::FTP2()
{
  SetApplicationType (Application::APPLICATION_TYPE_FTP2);
}

void
FTP2::DoStart (void)
{
  Simulator::Init()->Schedule(0.0, &FTP2::Send, this);
}

void
FTP2::DoStop (void)
{
}

void
FTP2::ScheduleTransmit (double time)
{
  if ( (Simulator::Init()->Now () + time) < GetStopTime () )
    {
      Simulator::Init()->Schedule(time, &FTP2::Send, this);
    }
}

void
FTP2::Send (void)
{

  int dataOfBurstAlreadySent = 0;

  for (int i = 0; i < GetSize() / MAXMTUSIZE; i++)
    {
      //CREATE A NEW PACKET (ADDING UDP, IP and PDCP HEADERS)
      Packet *packet = new Packet ();
  packet->SetID(GetID());
  SetID(GetID()+1);
      packet->SetTimeStamp (Simulator::Init()->Now ());
      packet->SetSize (MAXMTUSIZE);

      

      PacketTAGs *tags = new PacketTAGs ();
      tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_FTP2);
      tags->SetStartByte(dataOfBurstAlreadySent);
      tags->SetEndByte(dataOfBurstAlreadySent+MAXMTUSIZE-1);
      tags->SetApplicationSize (packet->GetSize ());
      dataOfBurstAlreadySent+=MAXMTUSIZE;
      packet->SetPacketTags(tags);


      UDPHeader *udp = new UDPHeader (GetClassifierParameters ()->GetSourcePort (),
                                      GetClassifierParameters ()->GetDestinationPort ());
      packet->AddUDPHeader (udp);

      IPHeader *ip = new IPHeader (GetClassifierParameters ()->GetSourceID (),
                                   GetClassifierParameters ()->GetDestinationID ());
      packet->AddIPHeader (ip);

      PDCPHeader *pdcp = new PDCPHeader ();
      packet->AddPDCPHeader (pdcp);
      Trace (packet);
      GetRadioBearer()->Enqueue (packet);
    }

  uint16_t sizetosend = GetSize() % MAXMTUSIZE;

  if (sizetosend > 0)
    {
      //CREATE A NEW PACKET (ADDING UDP, IP and PDCP HEADERS)
      Packet *packet = new Packet ();
  packet->SetID(GetID());
  SetID(GetID()+1);
      packet->SetTimeStamp (Simulator::Init()->Now ());
      packet->SetSize (sizetosend);

      

      PacketTAGs *tags = new PacketTAGs ();
      tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_FTP2);
      tags->SetStartByte(dataOfBurstAlreadySent);
      tags->SetEndByte(dataOfBurstAlreadySent+sizetosend-1);
      tags->SetApplicationSize (packet->GetSize ());
      dataOfBurstAlreadySent+=sizetosend;
      packet->SetPacketTags(tags);


      UDPHeader *udp = new UDPHeader (GetClassifierParameters ()->GetSourcePort (),
                                      GetClassifierParameters ()->GetDestinationPort ());
      packet->AddUDPHeader (udp);

      IPHeader *ip = new IPHeader (GetClassifierParameters ()->GetSourceID (),
                                   GetClassifierParameters ()->GetDestinationID ());
      packet->AddIPHeader (ip);

      PDCPHeader *pdcp = new PDCPHeader ();
      packet->AddPDCPHeader (pdcp);
      
      Trace (packet);
      GetRadioBearer()->Enqueue (packet);
    }

  double interval = -log((double )rand()/RAND_MAX)*GetAverageInterval ();
  //round to milliseconds
  interval = round(interval*1000)/1000;

  ScheduleTransmit (interval);

}


int
FTP2::GetSize (void) const
{
  return m_size;
}

void
FTP2::SetSize(int size)
{
  m_size = size;
}
void
FTP2::SetAverageInterval(double interval)
{
  m_averageInterval = interval;
}

double
FTP2::GetAverageInterval (void) const
{
  return m_averageInterval;
}

