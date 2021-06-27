#pragma once
#include "VEM.h"
enum eManeuver {KL, LCL, LCR};
class BehaviorPlanner
{
    public:
    BehaviorPlanner(VEMPtr env);
    void updateEnvironment();
    void updateIntent();
    int m_previousEgoLane = -1;
    std::vector<int> m_lanechangewishVal;
    VEMPtr m_env;
    bool m_laneChangeWish = false;

};