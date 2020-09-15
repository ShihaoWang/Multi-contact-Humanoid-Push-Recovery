#include "CommonHeader.h"

CubicSplineInfo EndEffectorPathComputation(const Robot & SimRobot, const SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj){
  int SwingLinkInfoIndex = SimParaObj.getSwingLinkInfoIndex();
  CubicSplineInfo CubicSplineInfoObj = CubicSplineInfoObjComputation(SimRobot, SwingLinkInfoIndex, SelfCollisionInfoObj, SimParaObj);

  if(CubicSplineInfoObj.getReadyFlag()){
    const int PathWayPointsSize = 51;
    std::vector<Vector3> PathWayPoints(PathWayPointsSize);
    double sUnit = 1.0/(1.0 * PathWayPointsSize - 1.0);
    int TransitionIndex = 0;
    for (int i = 0; i < PathWayPointsSize; i++){
      double s = 1.0 * i * sUnit;
      Vector3 SplinePoint = CubicSplineInfoObj.s2Pos(s);
      PathWayPoints[TransitionIndex] = SplinePoint;
      TransitionIndex++;
    }
    SimParaObj.DataRecorderObj.setPathWaypoints(PathWayPoints);
  }
  return CubicSplineInfoObj;
}
