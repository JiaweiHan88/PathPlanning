#include "VEM.h"
#include "iostream"
#include "helpers.h"
VEM::VEM()
{
    auto lane0 = Lane(0);
    auto lane1 = Lane(1);
    auto lane2 = Lane(2);
    m_Lanes.emplace(lane0.m_ID, lane0);
    m_Lanes.emplace(lane1.m_ID, lane1);
    m_Lanes.emplace(lane2.m_ID, lane2);
}
void VEM::update(json &j)
{
    //reset previous values todo:: instead reset, update vehicles using vehicle ID
    m_EgoVeh.m_preVeh = nullptr;
    m_EgoVeh.m_RpreVeh = nullptr;
    m_EgoVeh.m_LpreVeh = nullptr;
    m_EgoVeh.m_sucVeh = nullptr;
    m_EgoVeh.m_LsucVeh = nullptr;
    m_EgoVeh.m_RsucVeh = nullptr;
    m_Lanes.at(0).m_laneVehicles.clear();
    m_Lanes.at(1).m_laneVehicles.clear();
    m_Lanes.at(2).m_laneVehicles.clear();
    m_vehicles.clear();

    //updatevalues
    m_EgoVeh.m_x = j["x"];
    m_EgoVeh.m_y = j["y"];
    m_EgoVeh.m_s = j["s"];
    m_EgoVeh.m_d = j["d"];
    m_EgoVeh.m_yaw = j["yaw"];
    m_EgoVeh.m_yaw_rad = Helpers::getInstance().deg2rad(m_EgoVeh.m_yaw);
    m_EgoVeh.m_speed_abs = double(j["speed"]) / 2.24;
    m_EgoVeh.m_laneID = floor(m_EgoVeh.m_d / lane_width);
    //std::cout <<"EGO laneid "<< m_EgoVeh.m_laneID<<std::endl;
    auto surrounding_cars = j["sensor_fusion"];
    for (auto surcar : surrounding_cars)
    {
        //create vehicle object of each surrounding car
        auto pcar = std::make_shared<Vehicle>();
        pcar->m_id = surcar[0];
        pcar->m_x = surcar[1];
        pcar->m_y = surcar[2];
        pcar->m_vx = surcar[3];
        pcar->m_vy = surcar[4];
        pcar->m_speed_abs = sqrt(pcar->m_vx * pcar->m_vx + pcar->m_vy * pcar->m_vy);
        pcar->m_s = surcar[5];
        pcar->m_d = surcar[6];
        pcar->m_laneID = floor(pcar->m_d / lane_width);
        double ego_dist = fabs(m_EgoVeh.m_s - pcar->m_s);
        //only consider cars in sensor range, here we use 150 meters
        if (ego_dist > 150.0)
            continue;
        //only consider car on the same side as ego vehicle
        if (pcar->m_laneID < 0 || pcar->m_laneID > 2)
            continue;
        m_Lanes.at(pcar->m_laneID).m_laneVehicles.push_back(pcar);

        //if car is in front of ego vehicle
        if (pcar->m_s > m_EgoVeh.m_s)
        {
            //if car is on the same lane
            if (pcar->m_laneID == m_EgoVeh.m_laneID)
            {
                //if there is no preceeding vehicle yet or the current preceeding vehicle is farer away then car
                if (!m_EgoVeh.m_preVeh || m_EgoVeh.m_preVeh->m_s > pcar->m_s)
                {
                    //reset preceeding vehicle
                    m_EgoVeh.m_preVeh = pcar;
                }
            }
            //if car is on the left side
            else if (pcar->m_laneID == m_EgoVeh.m_laneID - 1)
            {
                if (!m_EgoVeh.m_LpreVeh || m_EgoVeh.m_LpreVeh->m_s > pcar->m_s)
                {
                    m_EgoVeh.m_LpreVeh = pcar;
                }
            }
            //if car is on the right side
            else if (pcar->m_laneID == m_EgoVeh.m_laneID + 1)
            {
                if (!m_EgoVeh.m_RpreVeh || m_EgoVeh.m_RpreVeh->m_s > pcar->m_s)
                {
                    m_EgoVeh.m_RpreVeh = pcar;
                }
            }
        }
        else
        {
            //if car is on the same lane
            if (pcar->m_laneID == m_EgoVeh.m_laneID)
            {
                //if there is no preceeding vehicle yet or the current preceeding vehicle is farer away then car
                if (!m_EgoVeh.m_sucVeh || m_EgoVeh.m_sucVeh->m_s < pcar->m_s)
                {
                    //reset preceeding vehicle
                    m_EgoVeh.m_sucVeh = pcar;
                }
            }
            //if car is on the left side
            else if (pcar->m_laneID == m_EgoVeh.m_laneID - 1)
            {
                if (!m_EgoVeh.m_LsucVeh || m_EgoVeh.m_LsucVeh->m_s < pcar->m_s)
                {
                    m_EgoVeh.m_LsucVeh = pcar;
                }
            }
            //if car is on the right side
            else if (pcar->m_laneID == m_EgoVeh.m_laneID + 1)
            {
                if (!m_EgoVeh.m_RsucVeh || m_EgoVeh.m_RsucVeh->m_s < pcar->m_s)
                {
                    m_EgoVeh.m_RsucVeh = pcar;
                }
            }
        }

        m_vehicles.push_back(pcar);
    }
}
