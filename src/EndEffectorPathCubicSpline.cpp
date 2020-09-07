#include "CommonHeader.h"

// static CubicSplineInfo cSplineGene(const std::vector<Vector3> & Points, const int & SwingLinkInfoIndex, const double & SelfTol, SelfLinkGeoInfo & SelfLinkGeoObj, int & PointIndex, Vector3 & ShiftPoint, double & ShiftDist, bool & FeasibleFlag){
//   // Algorithm stops when tolerance cannot be satisfied!
//   CubicSplineInfo CubicSplineInfoObj(Points);
//   std::vector<Vector3> ShiftPointVec;
//   std::vector<double> ShiftPointDisVec;
//   std::vector<int> ShiftIndexVec;
//   const int PtNo = Points.size();
//   const int MagNo = 10;
//   const int TotalNo = MagNo * PtNo;
//   int sIndex = 0;
//   double sUnit = 1.0/(1.0 * TotalNo - 1.0);
//   for (int i = 0; i < PtNo; i++) {
//     for (int j = 0; j < MagNo; j++) {
//       double s = 1.0 * sIndex * sUnit;
//       Vector3 SplinePoint = CubicSplineInfoObj.s2Pos(s);
//       double CurEnvPtDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SplinePoint);
//       double CurSelfPtDist; Vector3 CurSelfPtGrad;
//       SelfLinkGeoObj.SelfCollisionDistNGrad(SwingLinkInfoIndex, SplinePoint, CurSelfPtDist, CurSelfPtGrad);
//       if((CurEnvPtDist<0)||(CurSelfPtDist<SelfTol)){
//         ShiftPointVec.push_back(SplinePoint);
//         double DistMetric = 0.0;
//         if(CurEnvPtDist<0)  DistMetric-=CurEnvPtDist;
//         if(CurSelfPtDist<SelfTol) DistMetric+=SelfTol - CurSelfPtDist;
//         ShiftPointDisVec.push_back(DistMetric);
//         ShiftIndexVec.push_back(i);
//       }
//       sIndex++;
//     }
//   }
//   switch (ShiftPointVec.size()){
//     case 0:
//     FeasibleFlag = true;
//     break;
//     default:{
//       FeasibleFlag = false;
//       int ShiftPointIndex = std::distance(ShiftPointDisVec.begin(), std::max_element(ShiftPointDisVec.begin(), ShiftPointDisVec.end()));
//       PointIndex = ShiftIndexVec[ShiftPointIndex];
//       ShiftPoint = ShiftPointVec[ShiftPointIndex];
//       ShiftDist = ShiftPointDisVec[ShiftPointIndex];
//     }
//     break;
//   }
//   return CubicSplineInfoObj;
// }


CubicSplineInfo CubicSplineInfoObjComputation(const Robot & SimRobot, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, const SimPara & SimParaObj){
  
  // This function is used to generate a collision-free path!
  std::vector<Vector3> WayPoints;
  std::vector<double> sVals; 

  InitialWayPointsComputation(SimRobot, SwingLinkInfoIndex, SimParaObj, WayPoints, sVals);
  Vector3Writer(WayPoints, "InitialPathWayPoints");
  bool InitialShifterFlag = true;

  WayPoints = InitialWayPointsShifter(WayPoints, SwingLinkInfoIndex, SelfCollisionInfoObj, InitialShifterFlag);
  Vector3Writer(WayPoints, "ShiftedPathWayPoints");

  CubicSplineInfo CubicSplineInfoObjEmpty;
  if(!InitialShifterFlag) return CubicSplineInfoObjEmpty;
  
  CubicSplineInfo CubicSplineInfoObj(WayPoints, sVals);
  // // Then the task is to generate a path which is collision-free.
  // const int TotalIter = 10;
  // int CurrentIter = 0;
  // while((FeasiFlag == false)&&(CurrentIter<=TotalIter)){
  //   Vector3 ShiftPoint;
  //   int ShiftPointIndex;
  //   double ShiftDist;
  //   CubicSplineInfoObj = cSplineGene(Points, SwingLinkInfoIndex, SelfTol, SelfLinkGeoObj, ShiftPointIndex, ShiftPoint, ShiftDist, FeasiFlag);
  //   if(!FeasiFlag){
  //     bool ShiftFlag;
  //     Vector3 NewShiftPoint = SinglePointShifter(ShiftPoint, SwingLinkInfoIndex, SelfTol, ShiftDist, SelfLinkGeoObj, ShiftFlag);
  //     if(!ShiftFlag)  return CubicSplineInfoObj;
  //     else {
  //       // Insert the NewShiftPoint back into Points vector
  //       std::vector<Vector3> NewPoints(Points.size()+1);
  //       for (int i = 0; i <= ShiftPointIndex; i++)
  //         NewPoints[i] =  Points[i];
  //       NewPoints[ShiftPointIndex+1] = NewShiftPoint;
  //       for (int i = ShiftPointIndex+1; i < Points.size(); i++)
  //         NewPoints[i+1] =  Points[i];
  //       Points = NewPoints;
  //     }
  //   }
  //   Vector3Writer(Points, "FineShiftedPathWayPoints");
  //   CurrentIter++;
  // }

  return CubicSplineInfoObj;
}