#include "Predictor.h"
#include "helpers.h"
#include <iostream>
Predictor::Predictor(VEMPtr env):
m_env(env)
{
}
bool Predictor::checkTargetLaneSafety()
{
    auto egoCar = m_env->m_EgoVeh;
    int currentLane = egoCar.m_laneID;
    int targetLane = egoCar.m_targetLaneID;
    double future_ego_d, future_ego_s;
    std::vector<Vehicle> candidates;
    for (auto vehicle : m_env->m_Lanes.at(currentLane).m_laneVehicles)
    {
        candidates.push_back(*vehicle);
    }
    if (currentLane != targetLane)
    {
        for (auto vehicle : m_env->m_Lanes.at(targetLane).m_laneVehicles)
        {
            candidates.push_back(*vehicle);
        }
    }
    
    future_ego_s = egoCar.m_speed_abs * m_predictionTimeStep + egoCar.m_s;
    for (auto candidate:candidates)
    {
        double future_s = candidate.m_speed_abs * m_predictionTimeStep + candidate.m_s;
        double rel_vel = fabs(egoCar.m_speed_abs - candidate.m_speed_abs);
        double safe_dist = 3 * rel_vel + 5;  
        if(fabs(future_s - future_ego_s) < safe_dist)
        {
            std::cout << "danger "<< future_s <<" "<< future_ego_s <<" "<< safe_dist <<std::endl;
            return false;
        }
    }
    return true;
}
