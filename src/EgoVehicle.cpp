#include "EgoVehicle.h"

EgoVehicle::EgoVehicle()
    : m_desiredSpeed(22.1),
      m_targetSpeed(0),
      m_laneChangeWish(false),
      m_targetLaneID(-1)
{
}
