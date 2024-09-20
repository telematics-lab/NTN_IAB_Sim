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
 * Author: Sergio Martiradonna <sergio.martiradonna@poliba.it>
 */


/*
 * 5G-air-simulator is the main program of the simulator.
 * To run simulations you can
 * (i) select one of example scenarios developed in Scenario/ ("./5G-air-simulator -h" for details)
 * (ii) create a new scenario and add its reference into the main program.
 *
 *  To create a new scenario, see documentation in DOC/ folder.
 *
 *  For any questions, please contact the author at
 *  g.piro@poliba.it
 */

#include "scenarios/relay-example-ul.h"
#include "scenarios/relay-sat-ul.h"
#include "scenarios/paper_relay-sat-ul.h"
#include "scenarios/paper_direct-sat-ul.h"
#include "scenarios/relay-example-dl.h"
#include "scenarios/relay-sat-dl.h"
#include "scenarios/simple.h"
#include "scenarios/single-cell-without-interference.h"
#include "scenarios/single-cell-with-interference.h"
#include "scenarios/single-cell-with-femto.h"
#include "scenarios/multi-cell.h"
#include "scenarios/single-cell-with-streets.h"
#include "scenarios/multi-cell-sinrplot.h"
#include "scenarios/itu-calibration.h"
#include "scenarios/urban-macrocell-itu.h"
#include "scenarios/rural-macrocell-itu.h"
#include "scenarios/f5g-uc1.h"
#include "scenarios/f5g-uc2.h"
#include "scenarios/f5g-uc6.h"
#include "scenarios/MMC1.h"
#include "scenarios/nb-cell.h"
#include "scenarios/test-demo1.h"
#include "scenarios/test-tri-sector.h"
#include "scenarios/test-multi-cell-tri-sector.h"
#include "scenarios/test-multicast.h"
#include "scenarios/test-mbsfn.h"
#include "scenarios/test-unicast.h"
#include "scenarios/nb-cell-sat.h"
//#include "scenarios/nb-cell-sat-conf-paper.h"
#include "scenarios/nb-cell-sat-esa.h"
#include "scenarios/nb-cell-test.h"
#include "scenarios/UC1_relay-ul.h"
#include "scenarios/UC1_relay-dl.h"
#include "scenarios/UC1_direct-ul.h"
#include "scenarios/UC1_direct-dl.h"
#include "scenarios/UC2_relay-ul.h"
#include "scenarios/UC2_relay-dl.h"
#include "scenarios/UC2_direct-ul.h"
#include "scenarios/UC2_direct-dl.h"
#include "scenarios/V_003_MULTI_UE/UC2_relay-ul.h"
#include "scenarios/V_004_SATELLITE/UC1_relay-dl.h"
#include "scenarios/V_004_SATELLITE/UC1_relay-ul.h"
#include "scenarios/V_004_SATELLITE/UC2_relay-dl.h"
#include "scenarios/V_004_SATELLITE/UC2_relay-ul.h"
#include "scenarios/P_001_REG/UC1_relay-ul.h"
#include "scenarios/P_001_REG/UC2_relay-ul.h"
#include "scenarios/P_002_SENSOR_DATA/UC1_direct-ul.h"
#include "scenarios/P_002_SENSOR_DATA/UC1_relay-ul.h"
#include "scenarios/P_003_BB_DATA/UC2_direct-ul.h"
#include "scenarios/P_003_BB_DATA/UC2_relay-ul.h"
#include "scenarios/P_004_SAT_NODE_SENSOR_BH/UC1_relay-ul.h"
#include "scenarios/P_005_SAT_DONOR_BB_BH/UC2_relay-ul.h"
#include "utility/help.h"
#include <iostream>
#include <queue>
#include <fstream>
#include <stdlib.h>
#include <cstring>
#include <fenv.h>

std::mt19937 commonGen(time(NULL));

int
main (int argc, char *argv[])
{

  // Raise a floating point error when some computation gives a NaN as result
//  feenableexcept(FE_ALL_EXCEPT & ~FE_INEXACT);

  if (argc > 1)
    {

      /* Help */
      if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "-H") || !strcmp(argv[1],
          "--help") || !strcmp(argv[1], "--Help"))
        {
          Help ();
          return 0;
        }

      /* Run simple scenario */
      if (strcmp(argv[1], "Simple")==0)
        {
          Simple ();
        }



      /* Run more complex scenarios */
        if (strcmp(argv[1], "RelayExampleUl") == 0)
        {
          RelayExampleUl();
        }
        if (strcmp(argv[1], "RelaySatExampleUl") == 0)
        {
          RelaySatExampleUl();
        }
        if (strcmp(argv[1], "RelayExampleDl") == 0)
        {
          RelayExampleDl();
        }
        if (strcmp(argv[1], "RelaySatExampleDl") == 0)
        {
          RelaySatExampleDl();
        }
        if (strcmp(argv[1], "PaperRelay") == 0)
        {
          PaperRelaySatExampleUl(argc, argv);
        }
        if (strcmp(argv[1], "UC1RelayUl") == 0)
        {
          UC1RelayUl(argc, argv);
        }
        if (strcmp(argv[1], "UC1RelayDl") == 0)
        {
          UC1RelayDl(argc, argv);
        }
        if (strcmp(argv[1], "UC1DirectUl") == 0)
        {
          UC1DirectUl(argc, argv);
        }
        if (strcmp(argv[1], "UC1DirectDl") == 0)
        {
          UC1DirectDl(argc, argv);
        }
        if (strcmp(argv[1], "UC2RelayUl") == 0)
        {
          UC2RelayUl(argc, argv);
        }
        if (strcmp(argv[1], "UC2RelayDl") == 0)
        {
          UC2RelayDl(argc, argv);
        }
        if (strcmp(argv[1], "UC2DirectUl") == 0)
        {
          UC2DirectUl(argc, argv);
        }
        if (strcmp(argv[1], "UC2DirectDl") == 0)
        {
          UC2DirectDl(argc, argv);
        }
        if (strcmp(argv[1], "V_003_MULTI_UE") == 0)
        {
          V_003_MULTI_UE(argc, argv);
        }
        if (strcmp(argv[1], "V_004_SATELLITE_1") == 0)
        {
          V_004_SATELLITE_1(argc, argv);
        }
        if (strcmp(argv[1], "V_004_SATELLITE_2") == 0)
        {
          V_004_SATELLITE_2(argc, argv);
        }
        if (strcmp(argv[1], "V_004_SATELLITE_3") == 0)
        {
          V_004_SATELLITE_3(argc, argv);
        }
        if (strcmp(argv[1], "V_004_SATELLITE_4") == 0)
        {
          V_004_SATELLITE_4(argc, argv);
        }
        if (strcmp(argv[1], "P_001_REG_1") == 0)
        {
          P_001_REG_1(argc, argv);
        }
        if (strcmp(argv[1], "P_001_REG_2") == 0)
        {
          P_001_REG_2(argc, argv);
        }
        if (strcmp(argv[1], "P_002_SENSOR_DATA_1") == 0)
        {
          P_002_SENSOR_DATA_1(argc, argv);
        }
        if (strcmp(argv[1], "P_002_SENSOR_DATA_2") == 0)
        {
          P_002_SENSOR_DATA_2(argc, argv);
        }
        if (strcmp(argv[1], "P_003_BB_DATA_1") == 0)
        {
          P_003_BB_DATA_1(argc, argv);
        }
        if (strcmp(argv[1], "P_003_BB_DATA_2") == 0)
        {
          P_003_BB_DATA_2(argc, argv);
        }
         if (strcmp(argv[1], "P_004_SAT_NODE_SENSOR_BH") == 0)
        {
          P_004_SAT_NODE_SENSOR_BH(argc, argv);
        }
         if (strcmp(argv[1], "P_005_SAT_DONOR_BB_BH") == 0)
        {
          P_005_SAT_DONOR_BB_BH(argc, argv);
        }
        if (strcmp(argv[1], "PaperDirect") == 0)
        {
          PaperDirectSatExampleUl(argc, argv);
        }
      if (strcmp(argv[1], "SingleCell")==0)
        {
          SingleCellWithoutInterference (argc, argv);
        }
        
      if (strcmp(argv[1], "nbCell-Sat")==0)
      {
    	  //cout << "Apertura scenario satellitare" << endl;
          nbCell_Satellite (argc, argv);
      }

//      if (strcmp(argv[1], "nbCell-Sat-Conf-Paper")==0)
//      {
//    	  nbCell_Satellite_Conf_Paper (argc, argv);
//      }

      if (strcmp(argv[1], "nbCell-Sat-ESA")==0)
      {
    	  nbCell_Satellite_ESA(argc, argv);
      }

      if (strcmp(argv[1], "SingleCellWithI")==0)
        {
          SingleCellWithInterference (argc, argv);
        }
      if (strcmp(argv[1], "MultiCell")==0)
        {
          MultiCell (argc, argv);
        }
      if (strcmp(argv[1], "SingleCellWithFemto")==0)
        {
          SingleCellWithFemto(argc, argv);
        }
      if (strcmp(argv[1], "SingleCellWithStreets")==0)
        {
          SingleCellWithStreets (argc, argv);
        }
      if (strcmp(argv[1], "MMC1")==0)
        {
          MMC1 (argc, argv);
        }
      if (strcmp(argv[1], "nbCell")==0)
        {
          nbCell (argc, argv);
        }
      if (strcmp(argv[1], "test-tri-sector")==0)
        {
          TestTriSector (argc, argv);
        }
      if (strcmp(argv[1], "test-multi-cell-tri-sector")==0)
        {
          TestMultiCellTriSector (argc, argv);
        }
      if (strcmp(argv[1], "itu-calibration")==0)
        {
          ItuCalibration (argc, argv);
        }
      if (strcmp(argv[1], "urban-macrocell-itu")==0)
        {
          UrbanMacrocellItu (argc, argv);
        }
      if (strcmp(argv[1], "rural-macrocell-itu")==0)
        {
          RuralMacrocellItu (argc, argv);
        }
      if (strcmp(argv[1], "test-multicast")==0)
        {
          TestMulticast (argc, argv);
        }
      if (strcmp(argv[1], "test-mbsfn")==0)
        {
          TestMbsfn (argc, argv);
        }
      if (strcmp(argv[1], "test-unicast")==0)
        {
          TestUnicast (argc, argv);
        }
      if (strcmp(argv[1], "f5g-uc1")==0)
        {
          f5g_50MbpsEverywhere (argc, argv);
        }
      if (strcmp(argv[1], "f5g-uc2")==0)
        {
          f5g_HighSpeedTrain (argc, argv);
        }
      if (strcmp(argv[1], "f5g-uc6")==0)
        {
          f5g_BroadcastServices (argc, argv);
        }
      if (strcmp(argv[1], "f5g-demo1")==0)
        {
          f5g_demo1 (argc, argv);
        }
      if (strcmp(argv[1], "nbCellTest")==0)
        {
          nbCellTest (argc, argv);
        }
    }
  else
    {
      Help ();
      return 0;
    }
}
