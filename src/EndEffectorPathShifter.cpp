#include "CommonHeader.h"

extern std::vector<LinkInfo> LinkInfoObj;
extern SDFInfo SDFInfoObj;

static std::vector<Vector3> WayPointsSampler(const CubicSplineInfo & CubicSplineInfoObj){
  std::vector<double> sVec = LinearSpace(0.0, 1.0, 51);
  std::vector<Vector3> WayPoints;
  for (int i = 0; i < sVec.size(); i++)
    WayPoints.push_back(CubicSplineInfoObj.s2Pos(sVec[i]));
  return WayPoints;
}

static Vector3 PointShifter(Vector3 WayPoint, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, double SelfCollisionShiftTol){
  // This function will only move the point along a collision-free direction for a distance.
  double ShifterDistUnit    = 0.01;    // 1 cm
  bool ViolationShifterFlag = false;
  Vector3 ShifterDirection; ShifterDirection.setZero();

  // From Self-Collision
  double SelfDist; Vector3 SelfDirection;
  SelfCollisionInfoObj.getSelfCollisionDistNGrad(SwingLinkInfoIndex, WayPoint, SelfDist, SelfDirection);

  // From Environment
  double EnviDist;
  EnviDist = SDFInfoObj.SignedDistance(WayPoint);
  Vector3 EnviDirection = SDFInfoObj.SignedDistanceNormal(WayPoint);

  if(SelfDist<SelfCollisionShiftTol){
    ShifterDirection+= SelfDirection;
    ViolationShifterFlag = true;
  }
  if(EnviDist<0.0){
    ShifterDirection+=EnviDirection;
    ViolationShifterFlag = true;
  }
  if(!ViolationShifterFlag){
    // Choose the most dangerous one to move then
    double SelfMetric = SelfDist - SelfCollisionShiftTol;
    double EnviMetric = EnviDist;
    if(SelfMetric<EnviMetric) ShifterDirection = SelfDirection;
    else ShifterDirection = EnviDirection;
  }
  ShifterDirection.setNormalized(ShifterDirection);
  WayPoint += ShifterDistUnit * ShifterDirection;

  return WayPoint;
}

static Vector3 ShifterDirectionComputation(const std::vector<double> & ShifterDistanceVec, const std::vector<Vector3> & ShifterDirectionVec){
  std::vector<double> EffectiveShifterDistanceVec;
  std::vector<Vector3> EffectiveShifterDirectionVec;
  for (int i = 0; i < ShifterDistanceVec.size(); i++){
    if(ShifterDistanceVec[i]<0.0){
      EffectiveShifterDistanceVec.push_back(ShifterDistanceVec[i]);
      EffectiveShifterDirectionVec.push_back(ShifterDirectionVec[i]);
    }
  }
  Vector3 ShifterDirection; ShifterDirection.setZero();
  for (int i = 0; i < EffectiveShifterDistanceVec.size(); i++){
    ShifterDirection+=exp(-EffectiveShifterDistanceVec[i]) * EffectiveShifterDirectionVec[i];
  }
  ShifterDirection.setNormalized(ShifterDirection);
  return ShifterDirection;
}

static bool SegmentCollisionEval(double sFirst, double sSecond, const CubicSplineInfo & CubicSplineInfoObj, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, double SelfCollisionTol, Vector3 & ShiftDirection){
  int SegmentSize = 5;
  std::vector<double> Segment = LinearSpace(sFirst, sSecond, SegmentSize);
  std::vector<double>   ShifterDistanceVec(SegmentSize); 
  std::vector<Vector3>  ShifterDirectionVec(SegmentSize);
  std::vector<Vector3>  SegmentWayPointsVec;
  bool SegmentCollisionFlag = false;
  for (int i = 0; i < SegmentSize; i++){
    
    double ShiftDistance = 0.0;
    Vector3 ShifterDirection(0.0, 0.0, 0.0);
    
    double sVal = Segment[i];

    Vector3 CurWayPoint = CubicSplineInfoObj.s2Pos(sVal);
    SegmentWayPointsVec.push_back(CurWayPoint);
    // From Self-Collision
    double SelfDist; Vector3 SelfGrad;
    SelfCollisionInfoObj.getSelfCollisionDistNGrad(SwingLinkInfoIndex, CurWayPoint, SelfDist, SelfGrad);
    if(SelfDist<SelfCollisionTol){
      // std::printf("SelfDist: %f\n", SelfDist);
      SegmentCollisionFlag = true;
      ShiftDistance+=SelfDist - SelfCollisionTol;
      ShifterDirection+=SelfGrad;
    } 
    // From Environment
    double EnviDist;
    EnviDist = SDFInfoObj.SignedDistance(CurWayPoint);
    if(EnviDist<0.0){
      // std::printf("EnviDist: %f\n", EnviDist);
      ShiftDistance+=EnviDist;
      Vector3 EnviGrad = SDFInfoObj.SignedDistanceNormal(CurWayPoint);
      ShifterDirection+=EnviGrad;
      SegmentCollisionFlag = true;
    } 
    ShifterDistanceVec[i] = ShiftDistance;
    ShifterDirectionVec[i] = ShifterDirection;
  }
  // std::vector<Vector3> WayPointsSample = WayPointsSampler(CubicSplineInfoObj);
  // Vector3Writer(WayPointsSample, "SegmentWayPoints");
  if(SegmentCollisionFlag)
    ShiftDirection = ShifterDirectionComputation(ShifterDistanceVec, ShifterDirectionVec);
  return SegmentCollisionFlag;
}

static CubicSplineInfo CubicSplineInfoAdjuster(int sIndex, CubicSplineInfo & CubicSplineInfoObj, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, double SelfCollisionTol, Vector3 AdjusterDirection){
  double ShifterDistUnit    = 0.01;    // 1 cm
  int AdjusterLimit         = 10;
  std::vector<Vector3> pVec = CubicSplineInfoObj.pVec;
  std::vector<double> sVec  = CubicSplineInfoObj.sVec;
  double sFirst  = sVec[sIndex];
  double sSecond = sVec[sIndex + 1];
  Vector3 AdjustPoint = pVec[sIndex+1];
  int AdjusterTime = 0;
  while (AdjusterTime<AdjusterLimit){
    AdjustPoint += ShifterDistUnit * AdjusterDirection;
    pVec[sIndex + 1] = AdjustPoint;
    CubicSplineInfoObj.CubicSplineInfoUpdate(pVec, sVec);
    Vector3 AdjusterDirection; 
    bool SegmentCollisionFlag = SegmentCollisionEval(sFirst, sSecond, CubicSplineInfoObj, SwingLinkInfoIndex, SelfCollisionInfoObj, SelfCollisionTol, AdjusterDirection);
    if(!SegmentCollisionFlag){
      CubicSplineInfoObj.setReadyFlag(true);
      return CubicSplineInfoObj;
    } else {
      AdjusterTime++;
    }
  }
  CubicSplineInfoObj.setReadyFlag(false);
  return CubicSplineInfoObj;
}

CubicSplineInfo InitialWayPointsShifter(CubicSplineInfo & CubicSplineInfoObj, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, const SimPara & SimParaObj, const int & EdgeSize){
  // Here the shifted is conducted to move these points outward if a collision is detected.
  std::vector<double> sVec = CubicSplineInfoObj.sVec;
  double SelfCollisionTol = SimParaObj.getSelfCollisionTol();
  for (int sIndex = EdgeSize; sIndex < sVec.size()-EdgeSize; sIndex++){
    double sFirst  = sVec[sIndex];
    double sSecond = sVec[sIndex + 1];
    Vector3 ShifterDirection; 
    bool SegmentCollisionFlag = SegmentCollisionEval(sFirst, sSecond, CubicSplineInfoObj, SwingLinkInfoIndex, SelfCollisionInfoObj, SelfCollisionTol, ShifterDirection);
    if(SegmentCollisionFlag){
      CubicSplineInfo CubicSplineInfoTemp = CubicSplineInfoAdjuster(sIndex, CubicSplineInfoObj, SwingLinkInfoIndex, SelfCollisionInfoObj, SelfCollisionTol, ShifterDirection);
      if(CubicSplineInfoTemp.getReadyFlag()){
        CubicSplineInfoObj = CubicSplineInfoTemp;
      } else {
        CubicSplineInfoObj.setReadyFlag(false);
        return CubicSplineInfoObj;
      }
    }
  }
  return CubicSplineInfoObj;
}