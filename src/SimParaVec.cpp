#include "CommonHeader.h"

std::vector<double> getSimParaVec(){
  double PushDuration     = 0.50;
  double DetectionWait    = 0.25;

  double TimeStep         = 0.025;
  double InitDuration     = 1.0;
  double TotalDuration    = 5.0;                     // Simulation lasts for 5s after initial duration

  double ForwardDurationSeed  = 0.5;
  double PhaseRatio           = 0.6;
  double ReductionRatio       = 0.5;

  double ContactSelectionCoeff= 1.25;

  std::vector<double> SimParaVec = {  PushDuration, DetectionWait, 
                                      TimeStep, InitDuration, TotalDuration, 
                                      ForwardDurationSeed, PhaseRatio, ReductionRatio, ContactSelectionCoeff};
  return SimParaVec;                                   
}

