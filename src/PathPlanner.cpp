#include "PathPlanner.h"
#include "helpers.h"
#include "spline.h"
#include <algorithm>
#include <iostream>
using std::vector;
PathPlanner::PathPlanner(VEMPtr env) : m_env(env)
{
    m_pred = std::make_shared<Predictor>(env);
    m_ref_vel = 0;
}

void PathPlanner::updatePreviousPath(json &j)
{
    // Previous path data given to the Planner
    m_previous_path_x = j["previous_path_x"];
    m_previous_path_y = j["previous_path_y"];

    // Previous path's end s and d values
    m_end_path_s = j["end_path_s"];
    m_end_path_d = j["end_path_d"];
}

void PathPlanner::updatePath()
{
    m_next_x_vals.clear();
    m_next_y_vals.clear();
    auto &egoCar = m_env->m_EgoVeh;

    //check whether we want to change lane and whether its possible to change lane
    bool change_lane = egoCar.m_laneChangeWish && m_pred->checkTargetLaneSafety(); // && (m_time_since_lastLC > 250);

    double target_d = egoCar.m_laneID * 4 + 2;
    if (change_lane)
    {
        target_d = egoCar.m_targetLaneID * 4 + 2;
    }

    //regardless whether we want to change lane or not, we have to adapt our velocity base on preceeding vehicle
    if (egoCar.m_preVeh)
    {
        double dist = 100;
        //safety distance = distance in m travelled in 2 seconds with the current velocity
        double safedist = std::max(egoCar.m_speed_abs * 2.0, 6.0);
        dist = fabs(egoCar.m_s - egoCar.m_preVeh->m_s);
        //if we are more then 2 times safety dist away, we can drive with max velocity;
        if (dist > 2 * safedist)
        {
            egoCar.m_targetSpeed = egoCar.m_desiredSpeed;
        }
        //else if we are between 2xsafety distance and 1x safety distance
        else if (dist > safedist)
        {
            tk::spline sp;
            sp.set_points(std::vector<double>{5, safedist, safedist * 2}, std::vector<double>{0, egoCar.m_preVeh->m_speed_abs, egoCar.m_desiredSpeed});
            egoCar.m_targetSpeed = std::min(sp(dist), egoCar.m_desiredSpeed);
        }
        else
        {
            egoCar.m_targetSpeed = egoCar.m_targetSpeed - 0.2;
        }
    }
    else
    {
        egoCar.m_targetSpeed = egoCar.m_desiredSpeed;
    }
    //adapt our reference velocity in small steps to keep acceleration under threshhold
    if (m_ref_vel < egoCar.m_targetSpeed)
    {
        m_ref_vel += 0.1;
    }
    else
    {
        m_ref_vel -= 0.1;
    }

    //define some anchor points for our spline trajectory
    vector<double> anchor_x;
    vector<double> anchor_y;

    //if we have some left over previous path points, we use them as starting anchor point to smooth our trajectory
    //else we use the current ego Car position and a calculated previous position as starting anchor points
    int previous_path_size = m_previous_path_x.size();
    double ref_x = egoCar.m_x;
    double ref_y = egoCar.m_y;
    double ref_yaw = egoCar.m_yaw_rad;
    double prev_car_x = egoCar.m_x - cos(egoCar.m_yaw_rad);
    double prev_car_y = egoCar.m_y - sin(egoCar.m_yaw_rad);
    if (previous_path_size >= 2)
    {
        ref_x = m_previous_path_x[previous_path_size - 1];
        ref_y = m_previous_path_y[previous_path_size - 1];
        prev_car_x = m_previous_path_x[previous_path_size - 2];
        prev_car_y = m_previous_path_y[previous_path_size - 2];
        ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
    }
    anchor_x.push_back(prev_car_x);
    anchor_x.push_back(ref_x);
    anchor_y.push_back(prev_car_y);
    anchor_y.push_back(ref_y);

    //define 3 additional anchor points based on the target lane;
    double anchor1_s_offset = std::max(egoCar.m_speed_abs * 2 + 5, 25.0);
    auto anchor1 = Helpers::getInstance().getXY(egoCar.m_s + anchor1_s_offset, target_d);
    auto anchor2 = Helpers::getInstance().getXY(egoCar.m_s + anchor1_s_offset + 30, target_d);
    auto anchor3 = Helpers::getInstance().getXY(egoCar.m_s + anchor1_s_offset + 60, target_d);

    anchor_x.push_back(anchor1[0]);
    anchor_x.push_back(anchor2[0]);
    anchor_x.push_back(anchor3[0]);
    anchor_y.push_back(anchor1[1]);
    anchor_y.push_back(anchor2[1]);
    anchor_y.push_back(anchor3[1]);

    //shift to car reference
    for (int i = 0; i < anchor_x.size(); i++)
    {
        double shift_x = anchor_x[i] - ref_x;
        double shift_y = anchor_y[i] - ref_y;

        anchor_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        anchor_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }
    tk::spline sp;
    sp.set_points(anchor_x, anchor_y);

    //fill trajectory with left over previous path points
    for (int i = 0; i < previous_path_size; i++)
    {
        m_next_x_vals.push_back(m_previous_path_x[i]);
        m_next_y_vals.push_back(m_previous_path_y[i]);
    }

    //split spline in appropriate x distance to get y values
    double target_x = anchor1_s_offset;
    double target_y = sp(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double N = target_dist / (0.02 * m_ref_vel);
    double x, y, x_temp, y_temp;
    for (int i = 1; i <= 50 - previous_path_size; i++)
    {
        //shift back to global coordinates and push to next path points
        x = i * (target_x / N);
        y = sp(x);
        x_temp = x;
        y_temp = y;

        x = x_temp * cos(ref_yaw) - y_temp * sin(ref_yaw) + ref_x;
        y = x_temp * sin(ref_yaw) + y_temp * cos(ref_yaw) + ref_y;
        m_next_x_vals.push_back(x);
        m_next_y_vals.push_back(y);
    }
}