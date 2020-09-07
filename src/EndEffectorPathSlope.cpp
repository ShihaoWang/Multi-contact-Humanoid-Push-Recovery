#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"


static Vector3 ContactDirectionParabolic(const Vector3 & PosInit, const Vector3 & PosGoal, const Vector3 & DirGoal){
  // This function calcualtes the initial spline direction such that the resulting path is a parabola.
  Vector3 Goal2Init = PosInit - PosGoal;
  Goal2Init.getNormalized(Goal2Init);
  double Proj = Goal2Init.dot(DirGoal);
  Vector3 Vert = DirGoal - Proj * Goal2Init;
  Vector3 DirInit = Vert - Proj * Goal2Init;
  return DirInit;
}

static Vector3 ContactDirectionVelocity(const Robot & SimRobot, int SwingLinkInfoIndex){
  // This function gets end effector's current velocity direction.
  Vector3 EndEffectorVelocity = getEndEffectorTranslationalVelocity(SimRobot, SwingLinkInfoIndex, SimRobot.dq);
  EndEffectorVelocity.setNormalized(EndEffectorVelocity);
  return EndEffectorVelocity;
}

void EndEffectorPathSlopeComputation(const Robot & SimRobot, int SwingLinkInfoIndex, Vector3 & PathInitSlope, Vector3 & PathEndSlope, const SimPara & SimParaObj){
  // Note that there the direction vector is a unit vector while a Slope is a scaled vector for a geometric path.
  // This function generates the transition path for robot's end effector.
  Vector3 InitContactPos        = SimParaObj.getInitContactPos();
  Vector3 GoalContactPos        = SimParaObj.getGoalContactPos();
  Vector3 GoalContactDirection  = SimParaObj.getGoalContactDirection(); 

  // Vector3 InitContactDirection  = ContactDirectionParabolic(InitContactPos, GoalContactPos, GoalContactDirection);
  Vector3 InitContactDirection  = ContactDirectionVelocity(SimRobot, SwingLinkInfoIndex);

  // An initial cubic spline is preferred to match end effector's current velocity direction.
  double SlopeScale = 0.25;
  PathInitSlope = SlopeScale * InitContactDirection;
  PathEndSlope  = -SlopeScale * GoalContactDirection;
  return;
}