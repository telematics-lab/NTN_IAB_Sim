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



#ifndef EXTERNALSOURCE_H_
#define EXTERNALSOURCE_H_

#include "Application.h"

class ExternalSource : public Application
{
public:
  ExternalSource();
  virtual ~ExternalSource() = default;

  virtual void DoStart (void);
  virtual void DoStop (void);

  void
  ScheduleTransmit (double time);
  void
  Send (Packet *p);

  double
  GetInterval (void) const;

private:

  double m_interval;
  int m_size;
};

#endif /* EXTERNALSOURCE_H_ */
