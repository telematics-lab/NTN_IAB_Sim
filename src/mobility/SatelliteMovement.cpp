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
 * Author: Antonio Petrosino  <antonio.petrosino@poliba.it>
 * Author: Giancarlo Sciddurlo  <giancarlo.sciddurlo@poliba.it>
 */

#include "SatelliteMovement.h"
#include "../componentManagers/NetworkManager.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../device/Gateway.h"
#include "../device/GNodeB.h"
#include "../device/UserEquipment.h"
#include "../load-parameters.h"
//#include "../core/cartesianCoodrdinates/CartesianCoordinates.h"
#include "../phy/BLERTrace/BLERvsSINR_NBIoT_SAT.h"
#include "../phy/BLERTrace/BLERvsSINR_5G_SAT.h"


SatelliteMovement::SatelliteMovement(int nSat, bool nb_iot)
{
SetNB_IoT(nb_iot);

  SetMobilityModel(Mobility::SATELLITE);
  SetSpeed (7059.22);
  SetSpeedDirection (0.0);
  SetPositionLastUpdate (0.0);
  SetHandover (false);
  SetLastHandoverTime (0.0);
  SetAbsolutePosition (nullptr);
  SetTimePositionUpdate(0.05);
  SetNumberOfSatellitePerOrbit(nSat);
  SetTimeOrbitPeriod(5676.98); // about 94 min
  //SetAntennaType(SatelliteMovement::PARABOLIC_REFLECTOR);
  SetAntennaType(SatelliteMovement::SAT1);
  //SetFixedAreaRadius(0);
  SetPointing(false);
  SetIsLeo(true);
  cout << "Generating satellite with the following parameters:"
	   << "\n\t Speed: " << GetSpeed() << " meter/second"
	   << "\n\t Number of satellite per Orbit: " << GetNumberOfSatellitePerOrbit()
	   << "\n\t Elapsed Time between 2 sat: " << GetTimeOrbitPeriod() / GetNumberOfSatellitePerOrbit() << " seconds"
	   << "\n ...Done!" << endl;
  Simulator::Init()->Schedule(m_gNBtimePositionUpdate,
                                &SatelliteMovement::UpdatePosition,
                                this,
                                Simulator::Init ()->Now());

}

SatelliteMovement::~SatelliteMovement()
{
  DeleteAbsolutePosition ();
}

void
SatelliteMovement::UpdatePosition (double time)
{
	//cout << "...t = "<<time <<" --> "<<endl;
DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG)
    cout << "\t START MOBILITY MODEL for "<< GetDevice ()->GetIDNetworkNode() << endl;
	cout << "UpdatePosition gNB satellitare avviato."<<endl;
DEBUG_LOG_END

  double timeInterval = time - GetPositionLastUpdate ();

  //if(timeInterval > 0.1 || time == 0.0){

DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG)
  cout << "MOBILITY_DEBUG: User ID: " << GetDevice ()->GetIDNetworkNode()
			<< "\n\t Cell ID " <<
			NetworkManager::Init()->GetCellIDFromPosition (GetAbsolutePosition()->GetCoordinateX(),
				GetAbsolutePosition()->GetCoordinateY())
			<< "\n\t Initial Position (X): " << GetAbsolutePosition()->GetCoordinateX()
			<< "\n\t Initial Position (Y): " << GetAbsolutePosition()->GetCoordinateY()
			<< "\n\t Speed: " << GetSpeed()
			<< "\n\t Speed Direction: " << GetSpeedDirection()
			<< "\n\t Time Last Update: " << GetPositionLastUpdate()
			<< "\n\t Time Interval: " << timeInterval
			<< endl;
DEBUG_LOG_END
DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG_TAB)
  cout <<  GetDevice ()->GetIDNetworkNode() << ""
			<< GetAbsolutePosition()->GetCoordinateX() << " "
			<< GetAbsolutePosition()->GetCoordinateY() << " "
			<< GetSpeed() << " "
			<< GetSpeedDirection() << " "
			<< GetPositionLastUpdate() << " "
			<< timeInterval << " "
			<< endl;
DEBUG_LOG_END

	double x_pos = GetSatPosition(time);
    // è espresso in metri al secondo se la velocita è data in km/h

	CartesianCoordinates *newPosition =
	new CartesianCoordinates(x_pos,
						   GetAbsolutePosition()->GetCoordinateY(),
						   GetAbsolutePosition()->GetCoordinateZ());
	newPosition->SetFloorHeight( GetAbsolutePosition()->GetFloorHeight() );

	SetAbsolutePosition(newPosition);
	SetPositionLastUpdate (time);

DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG)
  cout << "\n\t Final Position (X): " << GetAbsolutePosition()->GetCoordinateX()
			<< "\n\t Final Position (Y): " << GetAbsolutePosition()->GetCoordinateY()
			<< endl;
DEBUG_LOG_END

DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG_TAB)
  cout << GetAbsolutePosition()->GetCoordinateX() << " "
			<< GetAbsolutePosition()->GetCoordinateY()
			<< endl;
DEBUG_LOG_END

	Simulator::Init()->Schedule(m_gNBtimePositionUpdate,
                              &SatelliteMovement::UpdatePosition,
                              this,
                              Simulator::Init ()->Now());
  	delete newPosition;
  //}
}

int
SatelliteMovement::GetNumberOfSatellitePerOrbit(void){
    return m_numSatellitePerOrbit;
}

void
SatelliteMovement::SetNumberOfSatellitePerOrbit(int nSat){
    m_numSatellitePerOrbit = nSat;
}

double
SatelliteMovement::GetVisibilityPeriod()
{
	if (GetNumberOfSatellitePerOrbit() > 0){

		return GetTimeOrbitPeriod() / GetNumberOfSatellitePerOrbit();
	}
	else{
		cout << "ERROR: Number of satellite < 0. " << endl;
		return 0.0;
	}

}

double
SatelliteMovement::GetTimeOrbitPeriod(){
    return m_timeOrbitPeriod;
}

void
SatelliteMovement::SetTimeOrbitPeriod(double _period){
	m_timeOrbitPeriod =  _period;
}


void
SatelliteMovement::SetTimePositionUpdate(double _time){
	m_gNBtimePositionUpdate = _time;
}

double
SatelliteMovement::GetTimePositionUpdate(void){
	return m_gNBtimePositionUpdate;
}

double
SatelliteMovement::GetSatPosition (double time)
{
    if(GetNumberOfSatellitePerOrbit()>0){
        // metà_raggio_satellite [COSTANTE] - radius [INPUT]+ (velocità_relativa [calcolata in base ai 2 minuti] * tempo_visibilità_satellite_[modulo3000]])
        // metri, metri, m/s, secondi
        // modello basato su:
        // area                 = 30 ettari
        // altezza satellite     = 500km
        // tempo visibilità        = 111 secondi
        // periodicità sat.        = 1 ogni 2838 secondi

        double mod =  GetTimeOrbitPeriod() / GetNumberOfSatellitePerOrbit();
        double startOffset = 1039230;
        //double newPosition = -320000 - 309  +(7059.22 * (fmod(time,mod))) - start_offset;

        double newPosition = /*-GetSpotBeamRadius()*/ - GetFixedAreaRadius() +  (7059.22 * (fmod(time,mod))) - startOffset;
        //start_offset = 0;

        return newPosition;
    }else
    {
        cout <<"Number of satellite per orbit < 1." << endl;
        exit(1);
    }
}

bool SatelliteMovement::GetAttachProcedure(CartesianCoordinates *uePos)
{
	CartesianCoordinates *gnbPos = GetAbsolutePosition();

	// double distance = uePos->GetDistance3D (gnbPos); //Not used

	double ElAngle = GetElAngle(uePos);
	bool _attach = false;
	if (GetNB_IoT())
	{
		// Case NB-IoT use attach probability
		double snr4attachProbability = GetSNRfromElAngle_SAT(ElAngle, 2, GetAntennaType()); // snr for downlink
		double measuredRSRP = snr4attachProbability + GetTermalNoisePowerDB();

		double txPower = GetDevice()->GetPhy()->GetTxPower();
		double measuredCL = txPower - 30 - measuredRSRP;

		if (measuredCL > GetMCLthreshold())
		{
			return false;
		}

		double attachProbability = GetCellSelectionProb(snr4attachProbability);

		// std::uniform_int_distribution<> rndGen (0, 100);
		// extern std::mt19937 commonGen;
		// double randomNumber = rndGen(commonGen) / 100.;
		double randomNumber = (rand() % 100) / 100.;

		if (randomNumber < attachProbability)
		{
			_attach = true;
		}
	}
	else
	{
		// case 5G use minimum angle to match minimum SNR
		int scs = GetDevice()->GetPhy()->GetBandwidthManager()->GetSubcarrierSpacing();		 
			GNodeB *gnb = dynamic_cast<GNodeB *>(GetDevice());
		bool ground_type;
		if(gnb->GetUserEquipmentRecords()->at(0)->m_UE->GetIABNode() != nullptr){
			ground_type = false; 
		}else{
			ground_type = true; 
		}
		bool pointing = GetPointing();
		double MinElAngle = GetMinElAngleFromSNR5G(scs, ground_type, pointing, GetAntennaType());
		
		if (ElAngle >= MinElAngle)
		{
			_attach = true;
		}
	}
	return _attach;
}

	// Given the UE position, return the elevation angle
	double
	SatelliteMovement::GetElAngle(CartesianCoordinates * remoteObject)
	{
		if (m_isLeo)
		{
			CartesianCoordinates *gnbPos = GetAbsolutePosition();
			double distance = sqrt(pow((gnbPos->GetCoordinateX() - remoteObject->GetCoordinateX()), 2) +
								   pow((gnbPos->GetCoordinateY() - remoteObject->GetCoordinateY()), 2));

			double satHeight = gnbPos->GetCoordinateZ();
			double elangle = 0.0;

			if (distance > 0)
			{
				elangle = atan(satHeight / distance) * 180 / M_PI; // satHeight dovrebbe essere 500000 controllare
			}
			else if (distance == 0)
			{
				elangle = 90;
			}
			return elangle;
		}
		else
		{
			return m_GEOangle;
		}
	}

	double SatelliteMovement::GetDistanceFromAngle(double angle)
	{
		double distance=0.0;
		double h=35786000.0; //GN to Sat
		double B=6378000.0; //Earth radius
		double rad_angle = (angle/180.0)*M_PI;
		return sqrt(pow(B * cos(rad_angle), 2) + pow((h + B), 2) - pow(B, 2)) - B * cos(rad_angle);
	}

	// Return the position of the satellite given the elevation angle and the UE position
	double
	SatelliteMovement::GetSatPositionFromElAngle(CartesianCoordinates * remoteObject, double elangle)
	{
		CartesianCoordinates *gnbPos = GetAbsolutePosition();
		double satHeight = gnbPos->GetCoordinateZ();
		double distance = (satHeight) / (tan(elangle * M_PI / 180));
		double root = sqrt(pow(distance, 2) - pow(gnbPos->GetCoordinateY() - remoteObject->GetCoordinateY(), 2));

		double satPosition = remoteObject->GetCoordinateX() - root;
		double sat2Position = remoteObject->GetCoordinateX() + root;

		return satPosition;
	}

	// Given a destination, it returns the time needed to reach it
	double
	SatelliteMovement::GetTimeNeededForDestination(double satPosition)
	{
		double time = 0.0;
		double mod = GetTimeOrbitPeriod() / GetNumberOfSatellitePerOrbit();

		double gnbPosX = GetAbsolutePosition()->GetCoordinateX();
		double gnbPosXTarget = satPosition;
		double startOffset = 1039230;
		double speed = 7059.22;
        
		//Deprecated BeamRadius to set the elevation angle 
		double sMin = /*-GetSpotBeamRadius()*/ - GetFixedAreaRadius() + (speed * (fmod(0, mod))) - startOffset;
		double sMax = /*-GetSpotBeamRadius()*/ - GetFixedAreaRadius() + (speed * (fmod(mod - 0.0001, mod))) - startOffset;

		double space;
		if (gnbPosX >= gnbPosXTarget)
		{
			space = (sMax - gnbPosX) + (gnbPosXTarget - sMin);
		}
		else
		{
			space = gnbPosXTarget - gnbPosX;
		}

		time = space / speed;

		return time;
	}

	// This function returns the time needed to reach a position in wich the UE can be attached
	double
	SatelliteMovement::GetNextTimePositionUpdate(CartesianCoordinates * uePos)
	{
		double t;
		double ElAngle = GetElAngle(uePos);
		double MinElAngle;
		int scs = GetDevice()->GetPhy()->GetBandwidthManager()->GetSubcarrierSpacing();		
		GNodeB *gnb = dynamic_cast<GNodeB *>(GetDevice());
		bool ground_type;
		if(gnb->GetUserEquipmentRecords()->at(0)->m_UE->GetIABNode() != nullptr){
			ground_type = false; 
		}else{
			ground_type = true; 
		}
		bool pointing = GetPointing();

		if (m_nb_iot)
		{
			// If the scenario is NB-IoT:
			double minSNR = GetDevice()->GetPhy()->GetTxPower() - 30 - GetMCLthreshold() - GetTermalNoisePowerDB();
			MinElAngle = GetMinElAngleFromSNR(minSNR, GetAntennaType());
		}
		else
		{
			// Otherwise the scenario is 5G:
			MinElAngle = GetMinElAngleFromSNR5G(scs, ground_type, pointing, GetAntennaType());
		}

		if (ElAngle < MinElAngle)
		{
			t = GetTimeNeededForDestination(GetSatPositionFromElAngle(uePos, MinElAngle));
			if (t < 0.05)
				t = 0.05;
		}
		else
		{
			t = 0.05;
		}

		return t;
	}

	void
	SatelliteMovement::SetMCLthreshold(double SNRthreshold)
	{
		m_SNRthreshold = SNRthreshold;
	}

	double
	SatelliteMovement::GetMCLthreshold(void)
	{
		return m_SNRthreshold;
	}

	SatelliteMovement::AntennaType
	SatelliteMovement::GetAntennaType(void) const
	{
		return m_AntennaType;
	}

	void
	SatelliteMovement::SetAntennaType(AntennaType model)
	{
		m_AntennaType = model;
	}

	void
	SatelliteMovement::SetFixedAreaRadius(double radius)
	{
		m_fixedAreaRadius = radius;
		//cout << "Fixed area radius: " << m_fixedAreaRadius << " meters" << endl;
	}

	double
	SatelliteMovement::GetFixedAreaRadius(void)
	{
		return m_fixedAreaRadius;
	}

	double
	SatelliteMovement::GetSpotBeamRadius(void)
	{

		if (m_AntennaType == SatelliteMovement::PARABOLIC_REFLECTOR)
		{
			m_spotBeamRadius = 130000.0;
		}
		else if (m_AntennaType == SatelliteMovement::PATCH_ANTENNA)
		{
			m_spotBeamRadius = 320000.0;
		}
		else
		{
			cout << "Warning: no beamradius is defined for 5G ESA" << endl;
		}
        
		return m_spotBeamRadius;
	}

	void 
	SatelliteMovement::SetNB_IoT(bool nb_enable)
	{
		m_nb_iot = nb_enable;
	}
	
	bool 
	SatelliteMovement::GetNB_IoT(void)
	{
		return m_nb_iot;
	}

	bool 
	SatelliteMovement::GetPointing(void) const 
	{
		return m_pointing;
	};
	
	void 
	SatelliteMovement::SetPointing(bool p)
	{
		m_pointing = p;
	};

	void 
	SatelliteMovement::SetGEOangle(double angle)
	{
		m_GEOangle = angle;
	}

	double 
	SatelliteMovement::GetGEOangle(void)
	{
		return m_GEOangle;
	}

	void 
	SatelliteMovement::SetIsLeo(bool isleo)
	{
		m_isLeo = isleo;
	}

	bool 
	SatelliteMovement::GetIsLeo(void)
	{
		return m_isLeo;
	}
