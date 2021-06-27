#pragma once
#include "EgoVehicle.h"
class VEM;
class Lane
{
public:
    Lane(int ID);
    double getExpectedLaneVel(EgoVehicle &ego);
    int m_ID;
    VehicleVector m_laneVehicles;
};