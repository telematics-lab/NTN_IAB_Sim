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

#ifndef SAT_SIMPLE_ERRORMODEL_H_
#define SAT_SIMPLE_ERRORMODEL_H_

#include "error-model.h"
#include <vector>

class SatSimpleErrorModel : public ErrorModel
{
public:
  SatSimpleErrorModel() = default;
  virtual ~SatSimpleErrorModel() = default;

  virtual bool CheckForPhysicalError (vector<int> channels, vector<int> mcs, vector<double> m_sinr);
};

#endif /* SAT_SIMPLE_ERRORMODEL_H_ */
