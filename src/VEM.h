
#pragma once
#include "json.hpp"
#include "EgoVehicle.h"
#include "Lane.h"
using json = nlohmann::json;
class VEM;
typedef std::map <int, Lane> LaneMap;
typedef std::shared_ptr <VEM> VEMPtr;
class VEM
{
public:
    VEM();
    void update(json &j);

    VehicleVector m_vehicles;
    LaneMap m_Lanes;
    EgoVehicle m_EgoVeh;
    double lane_width = 4.0;
};