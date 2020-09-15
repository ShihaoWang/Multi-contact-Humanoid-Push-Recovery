#include "CommonHeader.h"
#include "RobotInfo.h"

static double ReductionMagnitude(double cur_s, double phase_s, double reduction_ratio){
  if(cur_s<phase_s) return 1.0;
  else{
    if(cur_s<=1.0){
      return 1.0 - (1.0 - reduction_ratio)/(1.0 - phase_s) * (cur_s - phase_s);
    } else return reduction_ratio;
  }
}

double StageTimeComputation(const Robot & SimRobot, const Config & CurConfig, const Config & CurVelocity,
                            const Config & UpdatedConfig, const std::vector<int> & SwingLinkChain, 
                            const double & sCur, const SimPara & SimParaObj,
                            std::vector<double> & NextConfig, 
                            std::vector<double> & NextVelocity){
  // Given Current Configuration, Velocity, and Updated Configuration, calculate the transition time.
  double StageTime = 0.0;
  double PhaseRatio = SimParaObj.PhaseRatio;
  std::vector<double> qMinVec = SimRobot.qMin;
  std::vector<double> qMaxVec = SimRobot.qMax;
  if(sCur<SimParaObj.PhaseRatio){
    StageTime = AccPhaseTimeComputation(CurConfig,  CurVelocity, 
                                        qMinVec, qMaxVec,
                                        NextConfig, NextVelocity,
                                        SimRobot.velMax, SimRobot.accMax,
                                        SwingLinkChain);                                       
  }
  else
  {
    double ReductionMag = ReductionMagnitude(sCur, PhaseRatio, SimParaObj.ReductionRatio);
    StageTime = DecPhaseTimeComputation(CurConfig,  CurVelocity, 
                                        qMinVec, qMaxVec,
                                        NextConfig, NextVelocity,
                                        SimRobot.velMax, SimRobot.accMax,
                                        SwingLinkChain, ReductionMag);
  }
  return StageTime;
}