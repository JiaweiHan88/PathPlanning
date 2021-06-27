#include "Lane.h"
#include "VEM.h"
Lane::Lane(int ID) : m_ID(ID)
{
}

double Lane::getExpectedLaneVel(EgoVehicle &ego)
{
    double averageVel = 0;
    int num_front_veh = 0;
    for (auto vehicle : m_laneVehicles)
    {
        double delta_s = vehicle->m_s - ego.m_s;
        if (delta_s > 0 && delta_s < 75)
        {
            num_front_veh++;
            averageVel += vehicle->m_speed_abs;
        }
    }
    if (num_front_veh)
        averageVel = (averageVel / num_front_veh);
    else
        averageVel = ego.m_desiredSpeed;
    return averageVel;
}
