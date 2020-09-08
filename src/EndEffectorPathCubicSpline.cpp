#include "CommonHeader.h"
extern SDFInfo SDFInfoObj;

static bool CubicSplineInfoEval(const CubicSplineInfo & CubicSplineInfoObj, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, double SelfCollisionTol, double & sVal){
  bool CubicSplineFeasiFlag = true;
  int sSize = CubicSplineInfoObj.sVec.size();
  std::vector<double> sVec = LinearSpace(0.0, 1.0, sSize);
  std::vector<double> DistMetrcVec;
  std::vector<double> sMetricVec;
  std::vector<Vector3> TestPathWayPoints;
  for (int i = 0; i < sSize; i++){
    Vector3 SplinePoint = CubicSplineInfoObj.s2Pos(sVec[i]);
    TestPathWayPoints.push_back(CubicSplineInfoObj.s2Pos(CubicSplineInfoObj.sVec[i]));
    double CurEnviDist = SDFInfoObj.SignedDistance(SplinePoint);
    double CurSelfDist = SelfCollisionInfoObj.getSelfCollisionDist(SwingLinkInfoIndex, SplinePoint);
    if((CurEnviDist<0)||(CurSelfDist<SelfCollisionTol)){
      double DistMetric = 0.0;
      if(CurEnviDist<0)  DistMetric-=CurEnviDist;
      if(CurSelfDist<SelfCollisionTol) DistMetric+=SelfCollisionTol - CurSelfDist;
      DistMetrcVec.push_back(DistMetric);
      sMetricVec.push_back(sVec[i]);
    }
  }
  // Vector3Writer(TestPathWayPoints, "TestPathWayPoints");

  if(sMetricVec.size()){
    int ShiftPointIndex = std::distance(DistMetrcVec.begin(), std::max_element(DistMetrcVec.begin(), DistMetrcVec.end()));
    sVal = sMetricVec[ShiftPointIndex];
    return false;
  } else return true;
}

CubicSplineInfo CubicSplineInfoObjComputation(const Robot & SimRobot, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, const SimPara & SimParaObj){
  
  // This function is used to generate a collision-free path!
  std::vector<Vector3>  WayPoints;
  std::vector<double>   sVals; 

  double SelfCollisionTol = SimParaObj.getSelfCollisionTol();

  InitialWayPointsComputation(SimRobot, SwingLinkInfoIndex, SimParaObj, WayPoints, sVals);
  // Vector3Writer(WayPoints, "InitialPathWayPoints");
  bool InitialShifterFlag = true;
  WayPoints = InitialWayPointsShifter(WayPoints, SwingLinkInfoIndex, SelfCollisionInfoObj, SimParaObj, InitialShifterFlag);
  // Vector3Writer(WayPoints, "ShiftedPathWayPoints");
  CubicSplineInfo CubicSplineInfoObjEmpty;
  if(!InitialShifterFlag) return CubicSplineInfoObjEmpty;
  
  CubicSplineInfo CubicSplineInfoObj(WayPoints, sVals);
  // Then the task is to generate a path which is collision-free.
  int IterLimit = 10;
  for (int i = 0; i < IterLimit; i++){
    double sVal; 
    bool CubicSplineFeasiFlag = 
    CubicSplineInfoEval(CubicSplineInfoObj, SwingLinkInfoIndex, SelfCollisionInfoObj, SimParaObj.getSelfCollisionTol(), sVal);
    if(CubicSplineFeasiFlag){
      CubicSplineInfoObj.setReadyFlag(true);
      break;
      return CubicSplineInfoObj;
    } else {
      Vector3 ShiftPoint = CubicSplineInfoObj.s2Pos(sVal);
      bool ShifterFlag = false;
      ShiftPoint = WayPointShifter(ShiftPoint, SwingLinkInfoIndex, SelfCollisionInfoObj, SimParaObj.getSelfCollisionShiftTol(), ShifterFlag);
      if(ShifterFlag){
        CubicSplineInfoObj.insert(ShiftPoint, sVal);
      } else {
        return CubicSplineInfoObj;
      }
    }
  }

  // std::vector<double> sPlot = LinearSpace(0.0, 1.0, 51);
  // std::vector<Vector3> Points(sPlot.size());
  // for (int i = 0; i < sPlot.size(); i++)
  //   Points[i] = CubicSplineInfoObj.s2Pos(sPlot[i]);
  
  // Vector3Writer(Points, "FineShiftedPathWayPoints");
  return CubicSplineInfoObj;
}