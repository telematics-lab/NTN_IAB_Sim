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


#include "gnb-phy.h"
#include "ue-phy.h"
#include "../device/NetworkNode.h"
#include "../channel/RadioChannel.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../phy/BLERTrace/BLERvsSINR_NBIoT_SAT.h"
#include "../phy/BLERTrace/BLERvsSINR_5G_SAT.h"
#include "../phy/nbiot-simple-error-model.h"
#include "../phy/sat-simple-error-model.h"
#include "../protocolStack/packet/packet-burst.h"
#include "../core/spectrum/transmitted-signal.h"
#include "../core/idealMessages/ideal-control-messages.h"
#include "../device/GNodeB.h"
#include "../device/UserEquipment.h"
#include "interference.h"
#include "error-model.h"
#include "../channel/propagation-model/propagation-loss-model.h"
#include "../protocolStack/mac/AMCModule.h"
#include "../utility/eesm-effective-sinr.h"
#include "../utility/miesm-effective-sinr.h"
#include "../componentManagers/FrameManager.h"
#include "../protocolStack/mac/nb-AMCModule.h"
#include "../device/CqiManager/cqi-manager.h"


#define UL_INTERFERENCE 4

GnbPhy::GnbPhy()
{
  SetDevice(nullptr);
  SetDlChannel(nullptr);
  SetUlChannel(nullptr);
  SetBandwidthManager(nullptr);
  SetTxSignal (nullptr);
  SetErrorModel (nullptr);
  SetInterference (nullptr);
  SetTxPower(33); //dBm
  SetTxAntennas(1);
  SetRxAntennas(1);
  GetAntennaParameters ()->SetType(Phy::AntennaParameters::ANTENNA_TYPE_OMNIDIRECTIONAL);
  GetAntennaParameters ()->SetBearing(0);
  GetAntennaParameters ()->SetEtilt(15);
}

GnbPhy::~GnbPhy()
{
  Destroy ();
}

void
GnbPhy::DoSetBandwidthManager (void)
{
  BandwidthManager* s = GetBandwidthManager ();
  vector<double> channels = s->GetDlSubChannels ();

  TransmittedSignal* txSignal = new TransmittedSignal ();

  vector< vector<double> > values;
  values.resize( GetTxAntennas () );

  double powerTx = pow (10., (GetTxPower () - 30) / 10); // in natural unit

  double txPower = 10 * log10 (powerTx / channels.size () ); //in dB
  for (int i = 0; i < GetTxAntennas (); i++)
    {
      for (auto channel : channels)
        {
          values.at(i).push_back(txPower);
        }
    }
  txSignal->SetValues (values);
  //txSignal->SetBandwidthManager (s->Copy());

  SetTxSignal (txSignal);
}

void
GnbPhy::StartTx (shared_ptr<PacketBurst> p)
{
  //cout << "Node " << GetDevice()->GetIDNetworkNode () << " starts phy tx" << endl;

  if (FrameManager::Init()->MbsfnEnabled()==true && FrameManager::Init()->isMbsfnSubframe()==true)
    {
      GetDlMcChannel ()->StartTx (p, GetTxSignal (), GetDevice ());
    }
  else
    {
      GetDlChannel ()->StartTx (p, GetTxSignal (), GetDevice ());
    }
}

void GnbPhy::StartRx(shared_ptr<PacketBurst> p, TransmittedSignal *txSignal)
{
    if (p->GetNPackets() == 0)
    {
      // Empty packet burst, return
      delete txSignal;
      return;
    }
    DEBUG_LOG_START_1(SIM_ENV_TEST_DEVICE_ON_CHANNEL)
    cout << "Node " << GetDevice()->GetIDNetworkNode() << " starts phy rx" << endl;
    DEBUG_LOG_END

    // COMPUTE THE SINR
    vector<double> rxSignalValues = txSignal->GetValues().at(0);
    
    vector<double> measuredSinr;
    vector<int> channelsForRx;
    

    // Note that for Uplink case no interference is considered
    double noise_interference = 10. * log10(pow(10., GetThermalNoise() / 10)); // dB

    bool satScenario = GetDevice()->GetPhy()->GetBandwidthManager()->GetSatenabled();
    double scs = GetDevice()->GetPhy()->GetBandwidthManager()->GetSubcarrierSpacing();
    bool nb_iotScenario = GetDevice()->GetPhy()->GetBandwidthManager()->GetNBIoTenabled();

    int chId = 0;
    for (auto power : rxSignalValues) // transmission power for the current sub channel [dB]
    {
      if (power != 0.)
      {
          channelsForRx.push_back(chId);
      
      chId++;

      if (satScenario)
      {
          if (nb_iotScenario)
          {
            measuredSinr.push_back(power - GetSatelliteNoisePowerDB());
          }
          else
          {
            measuredSinr.push_back(power - GetSatelliteSensitivity(scs));
          }
      }
      else
      {
          measuredSinr.push_back(power - noise_interference /*- UL_INTERFERENCE*/);
      }
      }
    }

    // CHECK FOR PHY ERROR
    bool phyError = false;

    if (GetErrorModel() != nullptr && p->GetNPackets() > 0 && satScenario)
    {
      if (nb_iotScenario)
      {
          vector<int> approxMCS;
          int MCS_ = FrameManager::Init()->GetMCSNBIoTSat();
          int RU_ = GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetNRUtoUE();

          UserEquipment *ue = GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE();
          approxMCS.push_back(((NBIoTSimpleErrorModel *)GetErrorModel())->GetRefMCS(MCS_, RU_));
          if (FrameManager::Init()->GetHARQ())
          {
            int TBS_ = GetDevice()->GetMacEntity()->GetNbAmcModule()->GetTBSizeFromMCS(MCS_, RU_);
            int requiredRx = GetMaxRequiredHARQretx(measuredSinr.at(channelsForRx.at(0)));

            // se il ritardo è superiore ad una certa soglia considero la trasmissione relativa al precedente satellite, discorso azzerato
            double delayHARQ = Simulator::Init()->Now() - GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetLastHARQTimestamp();
            if (delayHARQ > 400)
            {
              if (GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() > 0)
              {
                GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->SetHARQretx(0);
              }
            }

            GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->SetLastHARQTimestamp(Simulator::Init()->Now());

            if (GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() + 1 < requiredRx)
            {

              phyError = true;
              cout << "HARQ_ERR 1 RX_n: " << GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() + 1 << " / " << requiredRx << endl;
              GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->SetHARQretx(GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() + 1);
            }
            else
            {
              // phyError = false;
              phyError = ((NBIoTSimpleErrorModel *)GetErrorModel())->CheckForPhysicalErrorHARQ(channelsForRx, measuredSinr);
              cout << "HARQ_ERR " << phyError << " RX_n: " << GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() + 1 << " / " << requiredRx << endl;
              GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->SetHARQretx(0);
              // cout << "OK HARQ, non richiesta ritrasmissione." << endl;
            }
          }
          else
          {
            phyError = GetErrorModel()->CheckForPhysicalError(channelsForRx, approxMCS, measuredSinr);
          }

          if (_PHY_TRACING_)
          {
            CartesianCoordinates *uePos = ue->GetMobilityModel()->GetAbsolutePosition();
            CartesianCoordinates *gnbPos = GetDevice()->GetMobilityModel()->GetAbsolutePosition();

            double DIST_ = uePos->GetDistance3D(gnbPos);

            int TBS_ = GetDevice()->GetMacEntity()->GetNbAmcModule()->GetTBSizeFromMCS(MCS_, RU_);
            double EL_ = 0.0;
            // double EL_ = ue->GetMobilityModel()->GetAbsolutePosition() ->GetElAngle(GetDevice()->GetMobilityModel()->GetAbsolutePosition());
            if (GetDevice()->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE)
            {
              EL_ = ((SatelliteMovement *)GetDevice()->GetMobilityModel())->GetElAngle(ue->GetMobilityModel()->GetAbsolutePosition());
            }

            cout << "PHY_RX SRC " << ue->GetIDNetworkNode()
                 << " DST " << GetDevice()->GetIDNetworkNode()
                 << " DISTANCE " << DIST_
                 << " ELEVATION " << EL_
                 << " SNR " << measuredSinr.at(channelsForRx.at(0))
                 << " RU " << RU_
                 << " MCS " << MCS_
                 << " SIZE " << TBS_
                 << " ERR " << phyError
                 << " T " << Simulator::Init()->Now()
                 << endl;
          }
      }
      else
      {
          // For uplink Satellite scenario the MCS is static
          vector<int> staticMCS; // It's a vector for a compatibility reason
          // staticMCS.push_back(GetDevice()-> GetPhy()->GetBandwidthManager()->GetMCSindex());
          UserEquipment *ue;
          if (GetDevice()->GetUserEquipmentRecords()->at(0)->GetUE()->GetIABNode())
          {
            // IABNode to Satellite scenario, can I take the first one
            ue = GetDevice()->GetUserEquipmentRecords()->at(0)->GetUE();
          }
          else
          {
            // NTN Terminal to Satellite scenario, I have to take the right one
            ue = GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE();
          }
          staticMCS.push_back(((UePhy *)ue->GetPhy())->GetMCSforTx().at(0));
          phyError = GetErrorModel()->CheckForPhysicalError(channelsForRx, staticMCS, measuredSinr);

          if (_PHY_TRACING_)
          {
            CartesianCoordinates *uePos = ue->GetMobilityModel()->GetAbsolutePosition();
            CartesianCoordinates *gnbPos = GetDevice()->GetMobilityModel()->GetAbsolutePosition();
            double DIST_ = 0.0;
            AMCModule fake_amc = AMCModule();
            //int TBS_ = fake_amc.GetTBSizeFromMCS(staticMCS.at(0), channelsForRx.size(), 1);
            int TBS_ = fake_amc.GetTBSize5G(staticMCS.at(0), channelsForRx.size());
            double EL_ = 0.0;
            if (GetDevice()->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE)
            {
              
              if (((SatelliteMovement *)GetDevice()->GetMobilityModel())->GetIsLeo())
              {
                // IS LEO
                EL_ = ((SatelliteMovement *)GetDevice()->GetMobilityModel())->GetElAngle(uePos);
                DIST_ = uePos->GetDistance3D(gnbPos);
              }
              else
              {
                // IS GEO
                EL_ = ((SatelliteMovement *)GetDevice()->GetMobilityModel())->GetElAngle(uePos);
                DIST_ = ((SatelliteMovement *)GetDevice()->GetMobilityModel())->GetDistanceFromAngle(EL_);
              }
            }
            else if (ue->GetIABNode() != nullptr && ue->GetIABNode()->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE)
            {
              
              if (((SatelliteMovement *)ue->GetIABNode()->GetMobilityModel())->GetIsLeo())
              {
                // IS LEO
                EL_ = ((SatelliteMovement *)ue->GetIABNode()->GetMobilityModel())->GetElAngle(gnbPos);
                DIST_ = uePos->GetDistance3D(gnbPos);
              }
              else
              {
                // IS GEO
                EL_ = ((SatelliteMovement *)ue->GetIABNode()->GetMobilityModel())->GetGEOangle();
                DIST_ = ((SatelliteMovement *)ue->GetIABNode()->GetMobilityModel())->GetDistanceFromAngle(EL_);
              }
            }

            double effective_sinr = GetEesmEffectiveSinr(measuredSinr);
            cout << "PHY_RX SRC " << ue->GetIDNetworkNode()
                 << " DST " << GetDevice()->GetIDNetworkNode()
                 << " DISTANCE " << DIST_
                 << " ELEVATION " << EL_
                 << " SNR " << effective_sinr
                 << " RB " << channelsForRx.size()
                 << " MCS " << staticMCS.at(0)
                 << " SIZE " << TBS_
                 << " ERR " << phyError
                 << " T " << Simulator::Init()->Now()
                 << endl;
          }
      }
    }
    else
    {
      if (_PHY_TRACING_)
      {
          UserEquipment *ue;
          if (GetDevice()->GetUserEquipmentRecords()->at(0)->GetUE()->GetIABNode())
          {
            // IABNode to IABNode or gNB, can I take the first one
            ue = GetDevice()->GetUserEquipmentRecords()->at(0)->GetUE();
          }
          else
          {
            // UE to IABNode or gNB, I have to take the right one
            ue = GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE();
          }
          double effective_sinr = GetEesmEffectiveSinr(measuredSinr);
          int MCS_ = 0;
          int TBS_ = 0;
          if (channelsForRx.size() > 0)
          {
            CartesianCoordinates *uePos = ue->GetMobilityModel()->GetAbsolutePosition();
            CartesianCoordinates *gnbPos = GetDevice()->GetMobilityModel()->GetAbsolutePosition();
            double DIST_ = uePos->GetDistance3D(gnbPos);
            MCS_ = ((UePhy *)ue->GetPhy())->GetMCSforTx().at(0);
            //TBS_ = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(MCS_, channelsForRx.size(), 1);
            TBS_ = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule()->GetTBSize5G(MCS_, channelsForRx.size());
            cout << "PHY_RX SRC " << ue->GetIDNetworkNode()
                 << " DST " << ue->GetTargetNode()->GetIDNetworkNode()
                 << " DISTANCE " << DIST_
                 << " SINR " << effective_sinr
                 << " RB " << channelsForRx.size()
                 << " MCS " << MCS_
                 << " SIZE " << TBS_
                 << " T " << Simulator::Init()->Now();
            cout << endl;
          }
      }
    }

    if (!phyError && p->GetNPackets() > 0)
    {
      // FORWARD RECEIVED PACKETS TO THE DEVICE
      GetDevice()->ReceivePacketBurst(p);
    }

    delete txSignal;
}


void
GnbPhy::SendIdealControlMessage (IdealControlMessage *msg)
{
  GNodeB *gnb = GetDevice ();
  switch( msg->GetMessageType() )
    {
    case IdealControlMessage::ALLOCATION_MAP:
        {
          PdcchMapIdealControlMessage *pdcchMsg =  (PdcchMapIdealControlMessage*)msg;
          for (auto ue : pdcchMsg->GetTargetUEs() )
            {
              ue->GetPhy ()->ReceiveIdealControlMessage (msg);
            }
        }
      break;

    case IdealControlMessage::NB_IOT_ALLOCATION_MAP:
        {
          NbIoTMapIdealControlMessage *pdcchMsg =  (NbIoTMapIdealControlMessage*)msg;
          for (auto record : *pdcchMsg->GetMessage())
            {
              record.m_ue->GetPhy ()->ReceiveIdealControlMessage (msg);
            }
        }
      break;

    case IdealControlMessage::RA_RESPONSE:
    case IdealControlMessage::RA_CONNECTION_RESOLUTION:
      msg->GetDestinationDevice()->GetPhy ()->ReceiveIdealControlMessage (msg);
      break;

    default:
      cout << "Error in GnbPhy::SendIdealControlMessage: unknown message type (" << msg->GetMessageType() << ")" << endl;
      exit(1);
    }

  delete msg;
}

void
GnbPhy::ReceiveIdealControlMessage (IdealControlMessage *msg)
{
  GNodeB* gnb = GetDevice();
  gnb->GetMacEntity()->ReceiveIdealControlMessage(msg);
}

void GnbPhy::ReceiveReferenceSymbols(NetworkNode *n, TransmittedSignal *s)
{
  GNodeB::UserEquipmentRecord *user = ((UserEquipment *)n)->GetTargetNodeRecord();
  ReceivedSignal *rxSignal;
  if (GetUlChannel()->GetPropagationLossModel() != nullptr)
  {
      rxSignal = GetUlChannel()->GetPropagationLossModel()->AddLossModel(n, GetDevice(), s);
  }
  else
  {
      rxSignal = s->Copy();
  }
  AMCModule *amc = user->GetUE()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
  vector<double> ulQuality;
  vector<int> cqi;
  vector<double> rxSignalValues = rxSignal->GetValues().at(0);
  bool satScenario = GetDevice()->GetPhy()->GetBandwidthManager()->GetSatenabled();
  double scs = GetDevice()->GetPhy()->GetBandwidthManager()->GetSubcarrierSpacing();
  bool nb_iotScenario = GetDevice()->GetPhy()->GetBandwidthManager()->GetNBIoTenabled();
  delete rxSignal;
  double noise_interference = 10. * log10(pow(10., GetThermalNoise() / 10)); // dB

  vector<int> channelsForRx;
  int chId = 0;
  for (auto power : rxSignalValues)
  {
    
      if (power != 0.)
      {
              channelsForRx.push_back(chId);
              if (satScenario)
              {
            ulQuality.push_back(power - GetSatelliteSensitivity(scs));
              }
              else
              {
            ulQuality.push_back(power - noise_interference);
              }
              cqi.push_back(amc->GetCQIFromSinr(ulQuality.back()));
      }
      else
      {
              // If the channel is not assigned to the UE, the CQI is set to 0
              cqi.push_back(0);
      }
      chId++;
  }
  user->SetUplinkChannelStatusIndicator(cqi);

  delete s;
}

GNodeB*
GnbPhy::GetDevice(void)
{
  Phy* phy = (Phy*)this;
  return (GNodeB*)phy->GetDevice();
}
