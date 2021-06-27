#pragma once
#include "Vehicle.h"

class EgoVehicle: public Vehicle
{
public:
  public:
    EgoVehicle();

    std::shared_ptr<Vehicle>  m_preVeh;
    std::shared_ptr<Vehicle>  m_LpreVeh;
    std::shared_ptr<Vehicle>  m_RpreVeh;
    std::shared_ptr<Vehicle>  m_sucVeh;
    std::shared_ptr<Vehicle>  m_LsucVeh;
    std::shared_ptr<Vehicle>  m_RsucVeh;

    double m_targetSpeed; //calculated speed that the car should realize
    double m_desiredSpeed; //speed the car would like to realize
    bool m_laneChangeWish;
    int m_targetLaneID;
    int m_time_since_last_LC = 0;

};