#include "BehaviorPlanner.h"
#include "spline.h"
#include <iostream>
BehaviorPlanner::BehaviorPlanner(VEMPtr env) : m_env(env)
{

    m_lanechangewishVal.resize(3);
    m_lanechangewishVal[0] = 0;
    m_lanechangewishVal[1] = 0;
    m_lanechangewishVal[2] = 0;
}

void BehaviorPlanner::updateEnvironment()
{
    auto &egoCar = m_env->m_EgoVeh;

    //if a lanechange occured, reset lanechangewish values
    if (egoCar.m_laneID != m_previousEgoLane)
    {
        m_previousEgoLane = egoCar.m_laneID;
        m_lanechangewishVal[0] = 0;
        m_lanechangewishVal[1] = 0;
        m_lanechangewishVal[2] = 0;
    }
    //if no preceeding vehicle, we dont have intent to change lanes
    if (egoCar.m_preVeh)
    {
        double current_lane_vel = m_env->m_Lanes.at(egoCar.m_laneID).getExpectedLaneVel(egoCar);
        double rel_vel_offset = 2; //2m/s
        //calculate expected velocity of adjacent lanes
        for (int i = 0; i <= 2; i++)
        {
            //todo: for now only consider lanechange to adjacent lanes
            if (abs(i - egoCar.m_laneID) > 1)
                continue;
            //if any of the adjacent lanes have better expected velocity, increase lane change wish value
            double expectedVel = m_env->m_Lanes.at(i).getExpectedLaneVel(egoCar);
            if (expectedVel > current_lane_vel + rel_vel_offset)
            {
                m_lanechangewishVal[i]++;
            }
        }
    }
}

void BehaviorPlanner::updateIntent()
{
    auto &egoCar = m_env->m_EgoVeh;
    if (!egoCar.m_preVeh)
    {
        egoCar.m_laneChangeWish = false;
        std::cout << "No Preceeding Vehicle in range" << std::endl;
        return;
    }
    //find adjacent lane with highest velocity that we wish to change to
    double maxLaneVel = 0;
    int targetLane = egoCar.m_laneID;
    for (int i = 0; i <= 2; i++)
    {
        if (m_lanechangewishVal[i] > 90)
        {
            double laneVel = m_env->m_Lanes.at(i).getExpectedLaneVel(egoCar);
            if (laneVel > maxLaneVel)
            {
                maxLaneVel = m_env->m_Lanes.at(i).getExpectedLaneVel(egoCar);
                targetLane = i;
            }
        }
    }
    if (targetLane != egoCar.m_laneID)
    {
        std::cout << "lane change wish to laneID: " << targetLane << " because of higher vel of : " << maxLaneVel << std::endl;
        egoCar.m_laneChangeWish = true;
        egoCar.m_targetLaneID = targetLane;
    }
    else
    {
        std::cout << "No faster adjacent lane" << std::endl;
        egoCar.m_laneChangeWish = false;
    }
}
