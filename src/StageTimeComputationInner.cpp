#include "CommonHeader.h"
#include "RobotInfo.h"

double AccPhaseTimeComputation( const std::vector<double> & CurConfig,          const std::vector<double> & CurVelocity,
                                const std::vector<double> & ConfigLower,        const std::vector<double> & ConfigUpper,
                                std::vector<double> & NextConfig,               std::vector<double> & NextVelocity,
                                const std::vector<double> & VelocityBound,      const std::vector<double> & AccelerationBound,
                                const std::vector<int> & SwingLinkChain){
  std::vector<double> AccPhaseTimeTotal(SwingLinkChain.size());
  std::vector<double> SwingLinkChainVelocity(SwingLinkChain.size());
  for (int i = 0; i < SwingLinkChain.size(); i++) {
    int LinkIndex = SwingLinkChain[i];
    double InitPos = CurConfig[LinkIndex];
    double GoalPos = NextConfig[LinkIndex];
    double PosDiff = GoalPos - InitPos;
    double InitVelocity = CurVelocity[LinkIndex];
    double GoalVelocity;
    double VelBound = VelocityBound[LinkIndex];
    double AccBound = AccelerationBound[LinkIndex];
    double AccPhaseTime = AccPhaseTimeInner(PosDiff, InitVelocity, GoalVelocity, VelBound, AccBound);
    AccPhaseTimeTotal[i] = AccPhaseTime;
    SwingLinkChainVelocity[i] = GoalVelocity;
  }

  double AccTime = *max_element(AccPhaseTimeTotal.begin(), AccPhaseTimeTotal.end());
  for (int i = 0; i < SwingLinkChain.size(); i++){
    int LinkIndex = SwingLinkChain[i];
    double InitPos = CurConfig[LinkIndex];
    double GoalPos = NextConfig[LinkIndex];
    double PosDiff = GoalPos - InitPos;
    double InitVelocity = CurVelocity[LinkIndex];
    double GoalVelocity;
    double VelBound = VelocityBound[LinkIndex];
    double AccBound = AccelerationBound[LinkIndex];
    double qLower = ConfigLower[LinkIndex];
    double qUpper = ConfigUpper[LinkIndex];
    JointStateUpdate(InitPos, InitVelocity, GoalPos, GoalVelocity, qLower, qUpper, VelBound, AccBound, AccTime);
    NextConfig[LinkIndex] = GoalPos;
    NextVelocity[LinkIndex] = GoalVelocity;
  }
  return AccTime;
}

double DecPhaseTimeComputation( const std::vector<double> & CurConfig,      const std::vector<double> & CurVelocity,  
                                const std::vector<double> & ConfigLower,        const std::vector<double> & ConfigUpper,
                                std::vector<double> & NextConfig,           std::vector<double> & NextVelocity,
                                const std::vector<double> & VelocityBound,  const std::vector<double> & AccelerationBound,
                                const std::vector<int> & SwingLinkChain,    const double & ReductionRatio){
   // This function solves for the time in deceleration phase.
   std::vector<double> DampingVelocityBound(VelocityBound.size());
   for (int i = 0; i < DampingVelocityBound.size(); i++)
     DampingVelocityBound[i] = VelocityBound[i] * ReductionRatio;
   return AccPhaseTimeComputation(CurConfig, CurVelocity, ConfigLower, ConfigUpper, 
                                  NextConfig, NextVelocity, 
                                  DampingVelocityBound, AccelerationBound, 
                                  SwingLinkChain);
}