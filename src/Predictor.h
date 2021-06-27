#pragma once
#include "VEM.h"
class Predictor
{
public:
 Predictor(VEMPtr env);
 bool checkTargetLaneSafety();
 double m_predictionTimeStep = 1.5; //seconds
 VEMPtr m_env;
};