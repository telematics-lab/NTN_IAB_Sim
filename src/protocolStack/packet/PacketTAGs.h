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


#ifndef PACKETTAGS_H_
#define PACKETTAGS_H_

#include <vector>

class PacketTAGs
{
public:
  enum ApplicationType
  {
    APPLICATION_TYPE_VOIP,
    APPLICATION_TYPE_TRACE_BASED,
    APPLICATION_TYPE_INFINITE_BUFFER,
    APPLICATION_TYPE_CBR,
    APPLICATION_TYPE_WEB,
    APPLICATION_TYPE_FTP2,
    APPLICATION_TYPE_REGISTRATION
  };

  PacketTAGs ();
  PacketTAGs(ApplicationType type,
             int frameNumber,
             int startFrame,
             int endFrame);

  virtual ~PacketTAGs() = default;



  void
  SetApplicationType (ApplicationType type);
  ApplicationType
  GetApplicationType (void);

  //for video flows
  void
  SetFrameNumber (int frameNumber);
  int
  GetFrameNuber (void) const;

  void
  SetStartByte (int startByte);
  int
  GetStartByte (void) const;

  void
  SetEndByte (int endByte);
  int
  GetEndByte (void) const;

  void SetApplicationSize (int s);
  int GetApplicationSize ();
private:
  ApplicationType m_type;

  //for video flows
  int m_frameNumber;
  int m_startByte;
  int m_endBytes;

  int m_applicationSize;
};

#endif /* PACKETTAG_H_ */
