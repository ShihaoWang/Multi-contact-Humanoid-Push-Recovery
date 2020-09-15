#include "CommonHeader.h"

static double PositivePosNVel(  double PosDiff, 
                                const double & RawInitVelocity, double & GoalVelocity,
                                const double & VelocityBound,   const double & AccBound){
   // Two procedure => accelerate to velocity limit and remains there
   double InitVelocity = RawInitVelocity;
   double AccDist = (VelocityBound * VelocityBound - InitVelocity * InitVelocity)/(2.0 * AccBound);
   double AccTimeOffset = 0.0;
   if(AccDist<0.0){
     // This happens in velocity reduction stage where the initial velocity is larger than velocity bound.
     // As a result, the velocity needs to be dampened to be within boundary
     double AccDistOffset = -AccDist;
     PosDiff-=AccDistOffset;
     AccTimeOffset = (InitVelocity - VelocityBound)/AccBound;
     InitVelocity = VelocityBound;
     AccDist = 0.0;
   }
   if(AccDist>PosDiff){ // Does not need to accelerate to limit to reach GoalPos
     double AccTime = (-InitVelocity + sqrt(InitVelocity * InitVelocity + 2.0 * AccBound * PosDiff))/(1.0 * AccBound);
     GoalVelocity = InitVelocity + AccTime * AccBound;
     return AccTime + AccTimeOffset;
   }
   else { // This means that DOF accelerates to limit and then remain on maximum speed
     double AccTime1 = (VelocityBound - InitVelocity)/AccBound;
     double AccTime2 = (PosDiff - AccDist)/VelocityBound;
     double AccTime = AccTime1 + AccTime2;
     GoalVelocity = VelocityBound;
     return AccTime + AccTimeOffset;
   }
 }

 static double NegativePosNVel(  double PosDiff,
                                 const double & RawInitVelocity,  double & GoalVelocity,
                                 const double & VelocityBound,    const double & AccBound){
  double InitVelocity = RawInitVelocity;
  double AccDist = (VelocityBound * VelocityBound - InitVelocity * InitVelocity)/(2.0 * AccBound);
  double AccTimeOffset = 0.0;
  if(AccDist<0.0){
    double AccDistOffset = -AccDist;
    PosDiff+=AccDistOffset;
    AccTimeOffset = -(InitVelocity + VelocityBound)/AccBound;
    InitVelocity = -VelocityBound;
    AccDist = 0.0;
  }
  if(AccDist>-PosDiff){
    double AccTime = (InitVelocity + sqrt(InitVelocity * InitVelocity - 2.0 * AccBound * PosDiff))/(1.0 * AccBound);
    GoalVelocity = InitVelocity - AccTime * AccBound;
    return AccTime + AccTimeOffset;
  }
  else { // This means that DOF accelerates to limit and then remain on maximum speed
    double AccTime1 = (VelocityBound + InitVelocity)/AccBound;
    double AccTime2 = -(PosDiff + AccDist)/VelocityBound;
    double AccTime = AccTime1 + AccTime2;
    GoalVelocity = -VelocityBound;
    return AccTime + AccTimeOffset;
  }
}

double AccPhaseTimeInner( const double & PosDiff,
                          const double & InitVelocity,  double & GoalVelocity,
                          const double & VelocityBound, const double & AccBound){
  if(PosDiff>0.0){
    if(InitVelocity>0.0){
      return PositivePosNVel(PosDiff, InitVelocity, GoalVelocity, VelocityBound, AccBound);
    }
    else {
      double AccTime1 = -InitVelocity/AccBound;
      double AccDist1 = InitVelocity * InitVelocity/(2.0 * AccBound);
      double AccTime2 = PositivePosNVel(PosDiff + AccDist1, 0.0, GoalVelocity, VelocityBound, AccBound);
      return AccTime1 + AccTime2;
    }
  }
  else{
    if(InitVelocity<0.0){
      return NegativePosNVel(PosDiff, InitVelocity, GoalVelocity, VelocityBound, AccBound);
    } else {
      double AccTime1 = InitVelocity/AccBound;
      double AccDist1 = InitVelocity * InitVelocity/(2.0 * AccBound);
      double AccTime2 = NegativePosNVel(PosDiff - AccDist1, 0.0, GoalVelocity, VelocityBound, AccBound);
      return AccTime1 + AccTime2;
    }
  }
}