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

#include "single-uplink-packet-scheduler.h"
#include "../mac-entity.h"
#include "../../packet/Packet.h"
#include "../../packet/packet-burst.h"
#include "../../../device/NetworkNode.h"
#include "../../../flows/radio-bearer.h"
#include "../../../protocolStack/rrc/rrc-entity.h"
#include "../../../flows/application/Application.h"
#include "../../../device/GNodeB.h"
#include "../../../protocolStack/mac/AMCModule.h"
#include "../../../phy/phy.h"
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../core/idealMessages/ideal-control-messages.h"
#include "../../../flows/QoS/QoSParameters.h"
#include "../../../flows/MacQueue.h"
#include "../../../utility/eesm-effective-sinr.h"

SingleUplinkPacketScheduler::SingleUplinkPacketScheduler()
{
  SetMacEntity (nullptr);
  CreateUsersToSchedule ();
  
  m_roundRobinId = 0;
}

SingleUplinkPacketScheduler::~SingleUplinkPacketScheduler()
{
  Destroy ();
}

double
SingleUplinkPacketScheduler::ComputeSchedulingMetric (UserToSchedule* user, int subchannel)
{
  double metric;

  int channelCondition = user->m_channelContition.at (subchannel);
  AMCModule* amc = user->m_userToSchedule->GetProtocolStack()->GetMacEntity()->GetAmcModule();
  double spectralEfficiency = amc->GetEfficiencyFromCQI (channelCondition);
  metric = spectralEfficiency * 180000;

  return metric;
}

void SingleUplinkPacketScheduler::RBsAllocation()
{
  UsersToSchedule *users = GetUsersToSchedule();
  int nbOfRBs = GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();

  GnbRandomAccess *randomAccessManager = GetMacEntity()->GetRandomAccessManager();
  vector<int> reservedRBs = randomAccessManager->GetReservedSubChannels();

  // int nbPrbToAssign = 1;
  // int stop_nbOfRBs = users->size() + reservedRBs.size();

  // create a matrix of flow metrics
  vector<vector<double>> metrics;
  metrics.resize(nbOfRBs);
  for (int i = 0; i < nbOfRBs; i++)
  {
    metrics.at(i).resize(users->size());
    for (int j = 0; j < (int)users->size(); j++)
    {
      metrics.at(i).at(j) = ComputeSchedulingMetric(users->at(j), i);
    }
  }

  // RBs allocation
  int s = 0;
  vector<bool> *isScheduled = new vector<bool>(users->size());
  fill(isScheduled->begin(), isScheduled->end(), false);
  while (s < nbOfRBs)
  {
    if (find(reservedRBs.begin(), reservedRBs.end(), s) != reservedRBs.end())
    {
      // The RB is reserved
      s++;
      continue;
    }

    double targetMetric = 0;
    bool RBIsAllocated = false;
    UserToSchedule *scheduledUser;
    int scheduledUserIndex;

    // Search the user to schedule with the highest metric
    for (int k = 0; k < (int)users->size(); k++)
    {
      if (metrics.at(s).at(k) > targetMetric && isScheduled->at(k) == false)
      {
        targetMetric = metrics.at(s).at(k);
        RBIsAllocated = true;
        scheduledUser = users->at(k);
        scheduledUserIndex = k;
      }
    }
    if (RBIsAllocated)
    {
      AMCModule *amc = scheduledUser->m_userToSchedule->GetProtocolStack()->GetMacEntity()->GetAmcModule();
      vector<double> sinrs;
      for (auto c : scheduledUser->m_channelContition)
      {
        if(c != 0)
        sinrs.push_back(amc->GetSinrFromCQI(c));
      }
      int mcs=0;
      if(sinrs.size()!=0)
      mcs = amc->GetMCSFromSinrVector(sinrs);
      //int tbs = (amc->GetTBSizeFromMCS(mcs, 1)) / 8;
      int tbs = (amc->GetTBSize5G(mcs, 1)) / 8;
      scheduledUser->m_transmittedData = tbs;
      scheduledUser->m_listOfAllocatedRBs.push_back(s);
      s++;
      scheduledUser->m_selectedMCS = mcs;
      isScheduled->at(scheduledUserIndex) = true;
    }
    else
    {
      s++;
    }
  }
}
