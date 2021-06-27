#pragma once
#include "json.hpp"
#include <vector>

class Vehicle;
typedef std::shared_ptr<Vehicle> VehiclePtr;
typedef std::vector<VehiclePtr> VehicleVector;

using json = nlohmann::json;
class Vehicle
{
public:
  Vehicle();

  double m_s;
  double m_d;
  double m_vx;
  double m_vy;
  double m_x;
  double m_y;
  double m_yaw;
  double m_yaw_rad;
  int m_id;

  int m_laneID;
  double m_speed_abs;
};