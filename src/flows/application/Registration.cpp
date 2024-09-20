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



#include "Registration.h"
#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"
#include <random>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

Registration::Registration()
{
  SetApplicationType (Application::APPLICATION_TYPE_REGISTRATION);
  SetSent(0);
}

void
Registration::DoStart (void)
{
  Simulator::Init()->Schedule(0.0, &Registration::Send, this);
}

void
Registration::DoStop (void)
{
}

void
Registration::ScheduleTransmit (double time)
{
  Simulator::Init()->Schedule(time, &Registration::Send, this);
}

void
Registration::Send (void)
{
  //CREATE A NEW PACKET (ADDING UDP, IP and PDCP HEADERS)
  Packet *packet = new Packet ();
  packet->SetID(GetID());
  SetID(GetID()+1);
  packet->SetTimeStamp (Simulator::Init()->Now ());
  packet->SetSize (GetSize ());

  PacketTAGs *tags = new PacketTAGs ();
  tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_REGISTRATION);
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

  Trace (packet);

  SetSent(GetSent()+1);

  GetRadioBearer()->Enqueue (packet);

  if(GetSent()<GetNumber())
  ScheduleTransmit (GetInterval ());
  //else
  //m_nextApplication->Start();

}

int 
Registration::GetSize(void) const
{
  /*std::random_device rd{};
  std::mt19937 gen{rd()};
  double mean = GetMeanSize();
  double dev = GetDevSize();

  // values near the mean are the most likely
  // standard deviation affects the dispersion of generated values from the mean
  std::normal_distribution<double> d{mean, dev};

  // draw a sample from the normal distribution and round it to an integer
  int size = (int) abs(ceil(d(gen)));
  
  return size;*/
  return GetMeanSize();
}

int 
Registration::GetMeanSize(void) const
{  
  return m_size;
}

void
Registration::SetMeanSize(int size)
{
  m_size = size;
}

void
Registration::SetInterval(double interval)
{
  m_interval = interval;
}

double
Registration::GetInterval (void) const
{
  return m_interval;
}

void 
Registration::SetDevSize(int size) 
{
  m_dev = size;
}

int 
Registration::GetDevSize(void) const 
{
  return m_dev;
}

void 
Registration::SetSent(int sent) 
{
  m_sent = sent;
}

int 
Registration::GetSent(void) const
{
  return m_sent;
}

void 
Registration::SetNumber(double num) 
{
  m_number = num;
}

double
Registration::GetNumber(void) const 
{
  return m_number;
}

void
Registration::SetNextApplication(Application* a)
{
  m_nextApplication = a;
}
  
Application*
Registration::GetNextApplication (void) const
{
  return m_nextApplication;
}