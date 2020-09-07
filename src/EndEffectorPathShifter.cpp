#include "CommonHeader.h"

extern std::vector<LinkInfo> LinkInfoObj;
extern SDFInfo SDFInfoObj;

static Vector3 WayPointShifter(const Vector3 & WayPoint, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, bool & ShifterFlag){
  int    ShifterLimit       = 15;
  double SelfCollisionTol   = 0.025;    // 2.5 cm
  double ShifterDistUnit    = 0.005;    // 0.5 cm

  int    ShifterTime        = 0;
  Vector3 CurWayPoint       = WayPoint;

  bool SelfShifterFlag      = false;
  bool EnviShifterFlag      = false;
       ShifterFlag          = false;

  while(ShifterTime<ShifterLimit){
    
    Vector3 ShifterDirection;
    ShifterDirection.setZero();
    // From Self-Collision
    double SelfDist; Vector3 SelfGrad;
    SelfCollisionInfoObj.getSelfCollisionDistNGrad(SwingLinkInfoIndex, CurWayPoint, SelfDist, SelfGrad);
    // From Environment
    double EnviDist; Vector3 EnviGrad; EnviGrad.setZero();
    EnviDist = SDFInfoObj.SignedDistance(CurWayPoint);

    if(SelfDist<SelfCollisionTol){
      ShifterDirection+= SelfGrad;
      SelfShifterFlag = true;
    }
    if(EnviDist<0.0){
      Vector3 EnviDirection = SDFInfoObj.SignedDistanceNormal(CurWayPoint);
      ShifterDirection+=EnviDirection;
      EnviShifterFlag = true;
    }
    if((!SelfShifterFlag)&&(!EnviShifterFlag)){
      break;
    }
    ShifterDirection.setNormalized(ShifterDirection);
    CurWayPoint += ShifterDistUnit * ShifterDirection;
    ShifterTime++;
  }
  if((!SelfShifterFlag)&&(!EnviShifterFlag))
    ShifterFlag = true;
  return CurWayPoint;
}

std::vector<Vector3> InitialWayPointsShifter(const std::vector<Vector3> & WayPoints, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, bool & ShifterFlag){
  std::vector<Vector3> WayPointsNew;
  WayPointsNew.reserve(WayPoints.size());
  ShifterFlag = true;
  for (int i = 0; i < WayPoints.size(); i++){
    bool InnerShifterFlag = true;
    Vector3 WayPoint = WayPointShifter(WayPoints[i], SwingLinkInfoIndex, SelfCollisionInfoObj, InnerShifterFlag);
    if(InnerShifterFlag){
      WayPointsNew.push_back(WayPoint);
    } else {
      ShifterFlag = false;
      return WayPointsNew;
    }
  }
  return WayPointsNew;
}