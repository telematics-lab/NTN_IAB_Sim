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



#ifndef USEREQUIPMENT_H_
#define USEREQUIPMENT_H_

#include "NetworkNode.h"
#include "GNodeB.h"
#include "IABNode.h"
#include "../core/eventScheduler/event.h"
#include "../protocolStack/mac/ue-mac-entity.h"

class Gateway;
class CqiManager;


class UserEquipment : public NetworkNode
{
public:
  UserEquipment () = default;
  UserEquipment (int idElement,
                 double posx, double posy,
                 Cell *cell,
                 GNodeB* target,
                 bool handover, Mobility::MobilityModel model);
  UserEquipment (int idElement,
                 double posx, double posy, int speed, double speedDirection,
                 Cell *cell,
                 GNodeB* target,
                 bool handover, Mobility::MobilityModel model);
  UserEquipment(int idElement,
                double posx, double posy, int speed, double speedDirection,
                Cell *cell,
                GNodeB *target,
                bool handover, Mobility::MobilityModel model, IABNode *iabNode);

  virtual ~UserEquipment();

  void SetTargetNode (GNodeB *n);
  GNodeB* GetTargetNode (void);

  void SetIABNode(IABNode *n);
  IABNode *GetIABNode(void);

  void SetTargetNodeRecord (GNodeB::UserEquipmentRecord *r);
  GNodeB::UserEquipmentRecord* GetTargetNodeRecord (void);
  void UpdateUserPosition (double time);
  UeMacEntity* GetMacEntity(void) const;

  void SetCqiManager (CqiManager *cm);
  CqiManager* GetCqiManager (void);

  void SetTimePositionUpdate (double time);
  double GetTimePositionUpdate (void);

  void
  SetIndoorFlag ( bool flag );
  virtual bool
  IsIndoor (void);

  void SetLastActivity();

  void SetActivityTimeout(double timeout);
  double GetActivityTimeout();
  void SetRandomAccessType(UeRandomAccess::RandomAccessType type);

  void SetNRUtoUE(int _nru);
  int GetNRUtoUE(void);

  //Debug
  void Print (void);
    
  void SetTransmitting (bool);
  bool IsTransmitting (void);

  bool Attachment (void);
  void SetInactivity (void);

  void SetHARQretx(int _nRetx);
  int GetHARQretx(void);

  void SetLastHARQTimestamp(double now);
  double GetLastHARQTimestamp(void);

private:
  GNodeB* m_targetNode;
  GNodeB::UserEquipmentRecord* m_targetNodeRecord;
  CqiManager *m_cqiManager;

  bool m_isIndoor;
  int m_nCellSelInARow;
  double m_timePositionUpdate;
  double m_activityTimeout;
  shared_ptr<Event> m_activityTimeoutEvent;

  int m_assignedNRU;
  int m_HARQretx_done;
  bool m_isTransmitting;
  double m_HARQTimestamp;

  IABNode* m_attachedIAB; //If defined, the UE is a MT attached to an IAB
};

#endif /* USEREQUIPMENT_H_ */
