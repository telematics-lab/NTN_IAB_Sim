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
 * Author: Alessandro Grassi <alessandro.grassi@poliba.it>
 */


#include "propagation-loss-model.h"
#include "channel-realization.h"
#include "../../core/spectrum/bandwidth-manager.h"
#include "../../core/spectrum/transmitted-signal.h"
#include "../../load-parameters.h"
#include "../../device/NetworkNode.h"
#include "../../device/UserEquipment.h"
#include "../../phy/BLERTrace/BLERvsSINR_NBIoT_SAT.h"
#include "../../phy/BLERTrace/BLERvsSINR_5G_SAT.h"
#include <vector>
#include <iostream>
#include <math.h>


PropagationLossModel::~PropagationLossModel()
{
  m_channelRealizationMap.clear ();
}

void
PropagationLossModel::AddChannelRealization (ChannelRealization* chRealization)
{
  ChannelRealizationId_t idMap = make_pair (chRealization->GetSourceNode (),
                                 chRealization->GetDestinationNode ());
  m_channelRealizationMap.insert (
    pair <ChannelRealizationId_t, ChannelRealization* > (idMap, chRealization));
}

void
PropagationLossModel::DelChannelRealization (NetworkNode* src, NetworkNode* dst)
{
  ChannelRealizationId_t idMap = make_pair (src,dst);

  if (m_channelRealizationMap.find (idMap) != m_channelRealizationMap.end ())
    {
      m_channelRealizationMap.find (idMap)->second->Destroy ();
      m_channelRealizationMap.erase (idMap);
    }
}


ChannelRealization*
PropagationLossModel::GetChannelRealization (NetworkNode* src, NetworkNode* dst)
{
  ChannelRealizationId_t idMap = make_pair (src, dst);
  if (m_channelRealizationMap.count (idMap)>0)
    {
      return m_channelRealizationMap.find (idMap)->second;
    }
  else
    {
      return nullptr;
    }
}

PropagationLossModel::ChannelRealizationMap
PropagationLossModel::GetChannelRealizationMap (void)
{
  return m_channelRealizationMap;
}

// Given a signal transmitted from src to dst, it returns the received signal in wich losses are added (e.g. power is subtracted)
ReceivedSignal *
PropagationLossModel::AddLossModel(NetworkNode *src,
                                   NetworkNode *dst,
                                   TransmittedSignal *txSignal)
{

  DEBUG_LOG_START_1(SIM_ENV_TEST_PROPAGATION_LOSS_MODEL)
  cout << "\t  --> add loss between "
       << src->GetIDNetworkNode() << " and " << dst->GetIDNetworkNode() << endl;
  DEBUG_LOG_END

  ReceivedSignal *rxSignal = txSignal->Copy();

  /*
   * The loss propagation model is based on
   * a on a combination of four different models:
   * - the path loss
   * - the penetration loss
   * - the shadowind
   * - the multipath
   *
   * The rxPsd will be obtained considering, for each sub channel, the following
   * relations:
   * rxPsd (i) = txPsd (i) + m(i,t) - sh(i,t) - pnl(i,t) - pl (a,b);
   * where i is the i-th sub-channel and t is the current time (Simulator::Now()).
   */

  ChannelRealization *c = GetChannelRealization(src, dst);

  if (c != nullptr)
  {
      vector<vector<double>> rxSignalValues;
      rxSignalValues = rxSignal->GetValues();
      int nbOfTransmitAntennas;

      if (c->isMbsfnRealization())
      {
        nbOfTransmitAntennas = 1;
        rxSignal->SetIsMBSFNSignal(true);
        vector<vector<double>> temp(1, vector<double>(rxSignalValues[0].size(), 0));
        vector<vector<double>>::iterator row;
        vector<double>::iterator col;
        vector<vector<double>>::iterator i;
        vector<double>::iterator j;

        // sum the power from all the transmit antennas
        for (row = rxSignalValues.begin(); row != rxSignalValues.end(); row++)
        {
          i = temp.begin();
          j = i->begin();
          for (col = row->begin(); col != row->end(); col++)
          {
            (*j) = 10 * log10(pow(10., (*j) / 10) + pow(10., (*col) / 10));
            j++;
          }
        }

        rxSignalValues = temp;
        if (nbOfTransmitAntennas != (int)temp.size())
        {
          cout << "Error: TX signal dimension does not match the number of TX antennas" << endl;
          exit(1);
        }
      }
      else
      {
        nbOfTransmitAntennas = src->GetPhy()->GetTxAntennas();
        if (nbOfTransmitAntennas != (int)rxSignalValues.size())
        {
          cout << "Error: TX signal dimension does not match the number of TX antennas" << endl;
          exit(1);
        }
      }

      int nbOfReceiveAntennas = dst->GetPhy()->GetRxAntennas();
      rxSignalValues.resize(rxSignalValues.size() * nbOfReceiveAntennas);

      for (int i = nbOfTransmitAntennas * nbOfReceiveAntennas - 1; i > 0; i--)
      {
        /*
         * Warning: i/nbOfReceiveAntennas is an integer division, on purpose.
         * This loop copies each component of txSignalValues to "nbOfReceiveAntennas"
         * components of rxSignalValues.
         */
        rxSignalValues.at(i) = rxSignalValues.at(i / nbOfReceiveAntennas);
      }

      vector<vector<double>> loss = c->GetLoss();

      int direction;  //1 for uplink, 2 for downlink, 3 for downlink but from a MT
      bool satScenario = false;
      bool nb_iotScenario = false;
      // if source or destination is a satellite, then we are in a satellite scenario
      if ((src->GetNodeType()== NetworkNode::TYPE_RELAY_NODE || src->GetNodeType()== NetworkNode::TYPE_GNODEB) && src->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE)
      {
        satScenario = true;
        nb_iotScenario = ((SatelliteMovement *)src->GetMobilityModel())->GetNB_IoT();
        // The source is a satellite, the scenario is downlink
        direction = 2;
      }
      else if ((dst->GetNodeType()== NetworkNode::TYPE_RELAY_NODE || dst->GetNodeType()== NetworkNode::TYPE_GNODEB) && dst->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE)
      {
        satScenario = true;
        nb_iotScenario = ((SatelliteMovement *)dst->GetMobilityModel())->GetNB_IoT();
        // The destination is a satellite, the scenario is uplink
        direction = 1;
      }
      else if (src->GetNodeType()== NetworkNode::TYPE_UE && ((UserEquipment *)src)->GetIABNode() != nullptr )
      {
        if(((UserEquipment *)src)->GetIABNode()->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE){
        satScenario = true;
        // The source is a satellite MT, the scenario is downlink from a MT
        direction = 3;}
      }
      else if (dst->GetNodeType()== NetworkNode::TYPE_UE && ((UserEquipment *)dst)->GetIABNode() != nullptr )
      {
        if(((UserEquipment *)dst)->GetIABNode()->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE){
        satScenario = true;
        // The source is a terrestrial gNB, the scenario is uplink from a terrestrial gNB
        direction = 4;}
      }

      DEBUG_LOG_START_1(SIM_ENV_TEST_PROPAGATION_LOSS_MODEL)
      cout << "tx sub channels " << rxSignalValues.at(0).size() << " loss sub channels " << loss.at(0).size() << endl;
      DEBUG_LOG_END

      int nbOfPaths = rxSignalValues.size();
      int nbOfSubChannels = rxSignalValues.at(0).size();
      double rxPower = 0.0;
      double simulatedRxPower = 0.0;
      if (satScenario == true)
      {
         //double ElAngle = 0.0;

        if (nb_iotScenario == true)
        {
          double gain = 0;
          if (dst->GetPhy()->GetBandwidthManager()->GetSubcarrierSpacing() == 3.75)
          {
            gain = 16.8124;
          }
          else
          {
            if (dst->GetPhy()->GetBandwidthManager()->GetTones() == 1)
            {
              gain = 10.7918;
            }
          }

          if (direction == 1)
          {
            double ElAngle = ((SatelliteMovement *)src->GetMobilityModel())->GetElAngle(dst->GetMobilityModel()->GetAbsolutePosition());
            simulatedRxPower = GetRxPowerfromElAngle_SAT(ElAngle, ((SatelliteMovement *)src->GetMobilityModel())->GetAntennaType()) + gain;
          }
          else
          {
            double ElAngle = ((SatelliteMovement *)dst->GetMobilityModel())->GetElAngle(src->GetMobilityModel()->GetAbsolutePosition());
            simulatedRxPower = GetRxPowerfromElAngle_SAT(ElAngle, ((SatelliteMovement *)dst->GetMobilityModel())->GetAntennaType()) + gain;
          }
          //
        }
        else
        {
          //Retrieve RBs number
            int nbRB = 0;
            for (auto &row : rxSignalValues)
            {
              for (auto &col : row)
              {
                if (col != 0)
                {
                  nbRB++;
                }
              }
            }
          bool pointing = false;
          double ElAngle = 0.0;
          switch (direction)
          {
          case 1: // uplink
          {
            ElAngle = ((SatelliteMovement *)dst->GetMobilityModel())->GetElAngle(src->GetMobilityModel()->GetAbsolutePosition());
            pointing = ((SatelliteMovement *)dst->GetMobilityModel())-> GetPointing();
            // Check if source UE is an IAB Node:
            if (((UserEquipment *)src)->GetIABNode() == nullptr)
            {
              // For uplink case we have a fixed tx power. It's needed to spread on RBs.
              simulatedRxPower = GetRxPowerfromElAngle_SAT_5G(ElAngle, 1, true, pointing, ((SatelliteMovement *)dst->GetMobilityModel())->GetAntennaType()) - (10 * log10(nbRB));
            }
            else
            {
              // For uplink case we have a fixed tx power. It's needed to spread on RBs.
              simulatedRxPower = GetRxPowerfromElAngle_SAT_5G(ElAngle, 1, false, pointing, ((SatelliteMovement *)dst->GetMobilityModel())->GetAntennaType()) - (10 * log10(nbRB));
            }
          }
          break;
          case 2: // downlink
          {
            ElAngle = ((SatelliteMovement *)src->GetMobilityModel())->GetElAngle(dst->GetMobilityModel()->GetAbsolutePosition());
            pointing = ((SatelliteMovement *)src->GetMobilityModel())-> GetPointing();
            // Check if destination UE is an IAB Node:
            if (((UserEquipment *)dst)->GetIABNode() == nullptr)
            {
              // For downlink case we have a flexible tx power depending on bandwidth. No need to spread on RBs.
              simulatedRxPower = GetRxPowerfromElAngle_SAT_5G(ElAngle, 2, true, pointing, ((SatelliteMovement *)src->GetMobilityModel())->GetAntennaType());
            }
            else
            {
              // For downlink case we have a flexible tx power depending on bandwidth. No need to spread on RBs.
              simulatedRxPower = GetRxPowerfromElAngle_SAT_5G(ElAngle, 2, false, pointing, ((SatelliteMovement *)src->GetMobilityModel())->GetAntennaType());
            }
            break;
          }
          case 3: // reverse downlink
          {
            ElAngle = ((SatelliteMovement *)((UserEquipment *)src)->GetIABNode()->GetMobilityModel())->GetElAngle(dst->GetMobilityModel()->GetAbsolutePosition());
            pointing = ((SatelliteMovement *)((UserEquipment *)src)->GetIABNode()->GetMobilityModel())-> GetPointing();
            // For downlink case we have a flexible tx power depending on bandwidth. No need to spread on RBs.
            simulatedRxPower = GetRxPowerfromElAngle_SAT_5G(ElAngle, 2, false, pointing, ((SatelliteMovement *)((UserEquipment *)src)->GetIABNode()->GetMobilityModel())->GetAntennaType());
            break;
          }
          case 4: // reverse uplink
          {
            ElAngle = ((SatelliteMovement *)((UserEquipment *)dst)->GetIABNode()->GetMobilityModel())->GetElAngle(src->GetMobilityModel()->GetAbsolutePosition());
            pointing = ((SatelliteMovement *)((UserEquipment *)dst)->GetIABNode()->GetMobilityModel())-> GetPointing();
            // For uplink case we have a fixed tx power. It's needed to spread on RBs.
            simulatedRxPower = GetRxPowerfromElAngle_SAT_5G(ElAngle, 1, false, pointing, ((SatelliteMovement *)((UserEquipment *)dst)->GetIABNode()->GetMobilityModel())->GetAntennaType());
            break;
          }
          default:
          {
            cout << "Error: The scenario is 5G Sat but no valid direction are configured" << endl;
            exit(1);
            break;
          }
          }
        }

        for (auto &row : rxSignalValues)
        {
          for (auto &col : row)
          {
            if (col != 0)
            {
              //If the trasmitted power on this subchannel isn't zero, it is substituded with the simulated received one
              col = simulatedRxPower;
            }
          }
        }
      }

      for (int i = 0; i < nbOfPaths; i++)
      {
        for (int j = 0; j < nbOfSubChannels; j++)
        {

          if (satScenario == true)
          {
            rxPower = rxSignalValues.at(i).at(j); // add measured LB value
          }
          else
          {
            if (rxSignalValues.at(i).at(j) != 0)
            rxPower = rxSignalValues.at(i).at(j) + loss.at(i).at(j); // add propagation loss
            else
            rxPower = rxSignalValues.at(i).at(j);
          }

          DEBUG_LOG_START_1(SIM_ENV_TEST_PROPAGATION_LOSS_MODEL)
          cout << "\t\t path " << i << " sub channel = " << j
               << " rxSignalValues = " << rxSignalValues.at(i).at(j)
               << " loss = " << loss.at(i).at(j)
               << " rxPower = " << rxPower
               << endl;
          DEBUG_LOG_END

          rxSignalValues.at(i).at(j) = rxPower;
        }
      }
      rxSignal->SetValues(rxSignalValues);
      if (c->hasFastFading() == true)
      {
        rxSignal->SetPhases(c->GetPhases());
      }
  }
  else
  {
      cout << "ERROR: Channel Realization Not Found!" << endl;
      exit(1);
  }

  return rxSignal;
}
