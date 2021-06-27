#pragma once
#include "json.hpp"
#include "VEM.h"
#include "Predictor.h"
using json = nlohmann::json;
class PathPlanner
{
public:
    PathPlanner(VEMPtr env);
    void updatePreviousPath(json &j);
    void updatePath();
    json m_previous_path_x;
    json m_previous_path_y;
    double m_end_path_s;
    double m_end_path_d;
    double m_ref_vel= 0;
    int m_time_since_lastLC = 0;
    std::vector<double> m_next_x_vals;
    std::vector<double> m_next_y_vals;
    std::shared_ptr<Predictor> m_pred;
    VEMPtr m_env;
};