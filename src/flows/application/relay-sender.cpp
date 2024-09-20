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



#include "relay-sender.h"
#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"
#include <random>

RelaySender::RelaySender()
{
  SetApplicationType (Application::APPLICATION_TYPE_RELAY_SENDER);
}

void
RelaySender::DoStart (void)
{
}

void
RelaySender::DoStop (void)
{
}


void
RelaySender::Send (Packet *packet)
{
  packet->SetID(GetID());
  SetID(GetID()+1);
  Trace (packet);
  GetRadioBearer()->Enqueue (packet);
}


