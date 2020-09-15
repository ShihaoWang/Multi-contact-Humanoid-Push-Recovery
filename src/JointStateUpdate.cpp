#include "CommonHeader.h"

static void AccVioPosNVel2Time( double InitPos,       double & GoalPos,
                                double InitVelocity,  double & GoalVelocity,
                                double qLow,          double qUpp, 
                                double VelocityBound, double AccBound,
                                double DurationTime,  double AccEst){
  // Update GoalPos and GoalVelocity according to DurationTime when the violation is at acceleration level
  if(AccEst<=0){
    AccBound = -AccBound;
    VelocityBound = -VelocityBound;
  }
  double VelocityEst = InitVelocity + DurationTime * AccBound;
  if(VelocityEst * VelocityEst <=VelocityBound * VelocityBound){
    GoalPos = InitPos + (VelocityEst * VelocityEst - InitVelocity * InitVelocity)/(2.0 * AccBound);
    GoalVelocity = VelocityEst;
  }
  else{
      // Then the velocity first reaches maximum and then remains there for the rest of the time.
      double AccTime = (VelocityBound - InitVelocity)/AccBound;
      double RestTime = DurationTime - AccTime;
      double AccDist = (VelocityBound * VelocityBound - InitVelocity * InitVelocity)/(2.0 * AccBound);
      double FlatDist = GoalVelocity * RestTime;
      GoalPos = InitPos + AccDist + FlatDist;
      GoalVelocity = VelocityBound;
  }
  if(GoalPos<=qLow){
    GoalPos = qLow;
    GoalVelocity = 0.0;
  }
  if(GoalPos>=qUpp){
    GoalPos = qUpp;
    GoalVelocity = 0.0;
  }
  return;
}

static void VelVioPosNVel2Time( double InitPos,       double & GoalPos,
                                double InitVelocity,  double & GoalVelocity,
                                double qLow,          double qUpp,
                                double VelocityBound, double AccBound,
                                double DurationTime,  double AccEst){
  // Update GoalPos and GoalVelocity according to DurationTime when the violation is at velocity level so AccEst should be within AccBound
  if(AccEst<=0){
    AccBound = -AccBound;
    VelocityBound = -VelocityBound; 
  }
  // First compare the time needed to accelerate to velocity bound
  double AccTime = (VelocityBound - InitVelocity)/AccBound;
  if(AccTime<=DurationTime){
    // First to maximum and then remain there.
    double AccDist = (VelocityBound * VelocityBound - InitVelocity * InitVelocity)/(2.0 * AccBound);
    double RestTime = DurationTime - AccTime;
    double RestDist = RestTime * VelocityBound;
    GoalPos = InitPos + AccDist + RestDist;
    GoalVelocity = VelocityBound;
  } else {
    // Just need to accelerate to DurationTime then
    GoalVelocity = InitVelocity + AccBound * DurationTime;
    GoalPos = InitPos + (GoalVelocity * GoalVelocity - InitVelocity * InitVelocity)/(2.0 * AccBound);
  }
  if(GoalPos<=qLow){
    GoalPos = qLow;
    GoalVelocity = 0.0;
  }
  if(GoalPos>=qUpp){
    GoalPos = qUpp;
    GoalVelocity = 0.0;
  }
}

void JointStateUpdate(  const double & InitPos,       const double & InitVelocity, 
                        double & GoalPos,             double & GoalVelocity,
                        const double & qLow,          const double & qUpp,
                        const double & VelocityBound, const double & AccBound,
                        const double & DurationTime) {
  double AccTol = 1e-3;
  double PosDiff = GoalPos - InitPos;
  double AccEst = 2.0 * (PosDiff - InitVelocity * DurationTime)/(DurationTime * DurationTime);
  if(AccEst * AccEst<=AccBound * AccBound + AccTol){
    double VelocityEst = InitVelocity + AccEst * DurationTime;
    if(VelocityEst * VelocityEst<=VelocityBound * VelocityBound)
      GoalVelocity = VelocityEst;
    else{
      double MinDurationTime = AccPhaseTimeInner(PosDiff, InitVelocity, GoalVelocity, VelocityBound, AccBound);
      if(MinDurationTime<=DurationTime){
        if(VelocityEst>0) GoalVelocity = VelocityBound;
        else GoalVelocity = -VelocityBound;
      }
      else {
        VelVioPosNVel2Time( InitPos, GoalPos, InitVelocity, GoalVelocity,
                            qLow, qUpp, VelocityBound, AccBound, 
                            DurationTime, AccEst);
      }
    }
  }
  else {
    // This indicates that the current time is too large or too small.
    AccVioPosNVel2Time( InitPos, GoalPos, InitVelocity, GoalVelocity, 
                        qLow, qUpp, VelocityBound, AccBound, 
                        DurationTime, AccEst);
  }
  return;
}