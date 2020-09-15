#include "CommonHeader.h"
extern SDFInfo SDFInfoObj;

CubicSplineInfo CubicSplineInfoObjComputation(const Robot & SimRobot, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, const SimPara & SimParaObj){
  
  // This function is used to generate a collision-free path!
  std::vector<Vector3>  WayPoints;
  std::vector<double>   sVals; 

  double SelfCollisionTol = SimParaObj.getSelfCollisionTol();
  // SelfCollisionInfoObj.BBVerticesWriter();
  int EdgeSize;
  InitialWayPointsComputation(SimRobot, SwingLinkInfoIndex, SimParaObj, WayPoints, sVals, EdgeSize);
  // Vector3Writer(WayPoints, "InitialPathWayPoints");
  CubicSplineInfo CubicSplineInfoObj(WayPoints, sVals);
  CubicSplineInfoObj = InitialWayPointsShifter(CubicSplineInfoObj, SwingLinkInfoIndex, SelfCollisionInfoObj, SimParaObj, EdgeSize);
  CubicSplineInfo CubicSplineInfoObjEmpty;
  if(!CubicSplineInfoObj.getReadyFlag()) return CubicSplineInfoObjEmpty;
  
  std::vector<double> sPlot = LinearSpace(0.0, 1.0, 51);
  std::vector<Vector3> Points(sPlot.size());
  for (int i = 0; i < sPlot.size(); i++)
    Points[i] = CubicSplineInfoObj.s2Pos(sPlot[i]);
  Vector3Writer(Points, "FineShiftedPathWayPoints");

  return CubicSplineInfoObj;
}