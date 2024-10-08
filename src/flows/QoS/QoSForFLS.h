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


#ifndef QOSFORFLS_H_
#define QOSFORFLS_H_

#include "QoSParameters.h"

class QoSForFLS :public QoSParameters
{
public:
  QoSForFLS();
  virtual ~QoSForFLS();

  void
  SetNbOfCoefficients(int M);
  int
  GetNbOfCoefficients (void) const;

  void
  CreateQ (void);
  void
  CreateU (void);
  void
  CreateFilterCoefficients (void);

  int*
  GetQ (void);
  int*
  GetU (void);
  double*
  GetFilterCoefficients (void) const;

  void
  UpdateQ (int q_);
  void
  UpdateU (int u_);


  void
  SetDataToTransmit (int data);
  int
  GetDataToTransmit (void) const;
  void
  UpdateDataToTransmit (int data);


  //Debug:
  void
  Print (void);

private:
  int m_nbOfCoefficients;

  int *m_q;
  int *m_u;
  double *m_filterCoefficients;

  int m_dataToTransmit;

};

#endif /* QOSFORFLS_H_ */
