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

#include "roundrobin-uplink-packet-scheduler.h"
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

RoundRobinUplinkPacketScheduler::RoundRobinUplinkPacketScheduler()
{
  SetMacEntity (nullptr);
  CreateUsersToSchedule ();

  m_roundRobinId = 0;
}

RoundRobinUplinkPacketScheduler::~RoundRobinUplinkPacketScheduler()
{
  Destroy ();
}

double
RoundRobinUplinkPacketScheduler::ComputeSchedulingMetric (UserToSchedule* user, int subchannel)
{
  double metric;

  int channelCondition = user->m_channelContition.at (subchannel);
  AMCModule* amc = user->m_userToSchedule->GetProtocolStack()->GetMacEntity()->GetAmcModule();
  double spectralEfficiency = amc->GetEfficiencyFromCQI (channelCondition);
  metric = spectralEfficiency * 180000;

  return metric;
}

void RoundRobinUplinkPacketScheduler::RBsAllocation()
{
  UsersToSchedule *users = GetUsersToSchedule();
  int nbOfRBs = GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();

  GnbRandomAccess *randomAccessManager = GetMacEntity()->GetRandomAccessManager();
  vector<int> reservedRBs = randomAccessManager->GetReservedSubChannels();

  // Distrbute PRB in a fair way, if users->size() > nbOfRBs, the last users jump this scheduling phase
  int minRB = 5;
  int nbPrbToAssign = minRB; // Minimum number of PRB to assign
  // int stop_nbOfRBs = nbOfRBs;

  // If it's possible to assign more PRB, assign more PRB dividing them in a fair way
  if (( (int) ((nbOfRBs-reservedRBs.size()) /  users->size())) > nbPrbToAssign)
  {
    nbPrbToAssign = ceil((nbOfRBs-reservedRBs.size()) / users->size());
    // stop_nbOfRBs = nbPrbToAssign * users->size();
  }

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

    if (m_roundRobinId >= (int)users->size())
      m_roundRobinId = 0; // restart again from the beginning

    UserToSchedule *scheduledUser;
    if (isScheduled->at(m_roundRobinId) == false)
    {
     scheduledUser = users->at(m_roundRobinId);
    }
    else{break;}

    AMCModule *amc = scheduledUser->m_userToSchedule->GetProtocolStack()->GetMacEntity()->GetAmcModule();
    vector<double> sinrs;
    for (auto c : scheduledUser->m_channelContition)
    {
      if (c != 0)
        sinrs.push_back(amc->GetSinrFromCQI(c));
    }

      double effectiveSinr;

      if (sinrs.size() == 0)
      {
      effectiveSinr = -100;
      }
      else
      {
      effectiveSinr = GetEesmEffectiveSinr(sinrs); //The minimum is -4.95 (Due to CQI 1)
      }
      int mcs;
      if(amc->GetCQIFromSinr(effectiveSinr) == 1){ 
        //Channel condition are not so good, assign the minimum number of PRB
        nbPrbToAssign = minRB;
        mcs=0;
      }
      else
      {
      int nbPrbToAssign_1 = minRB;
      mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr + (10 * log10(sinrs.size())) - (10 * log10(nbPrbToAssign_1))));
      while (mcs > 1 && nbPrbToAssign_1 < nbPrbToAssign)
      {
      nbPrbToAssign_1 ++; //Increase the number of PRB to assign
      mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr + (10 * log10(sinrs.size())) - (10 * log10(nbPrbToAssign_1))));;
      }
      nbPrbToAssign = nbPrbToAssign_1;
      }

      //mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr - (10 * log10(nbPrbToAssign))));

 

    int nbPrbAssigned = 0;
    for (int i = 0; i < nbPrbToAssign; i++)
    {
      if (s + i < nbOfRBs)
      {
        scheduledUser->m_listOfAllocatedRBs.push_back(s + i);
        nbPrbAssigned++;
      }
    }

    // int tbs = ((amc->GetTBSizeFromMCS(mcs, nbPrbToAssign)) / 8);
    int tbs = ((amc->GetTBSize5G(mcs, nbPrbAssigned)) / 8);

    scheduledUser->m_transmittedData = tbs;
    if (nbPrbAssigned < nbPrbToAssign)
      {
        mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr - (10 * log10(nbPrbAssigned))));
        }

    scheduledUser->m_selectedMCS = mcs;

    s = s + nbPrbAssigned;
    isScheduled->at(m_roundRobinId) = true;
    m_roundRobinId++;
  }
}
