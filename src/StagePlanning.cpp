#include "CommonHeader.h"
#include "RobotInfo.h"

static double ProjectionLength(double cur_s, const Vector3 & InitDir, const Vector3 & GoalDir){
  // This function is mainly used for hand contact planning.
  double x = GoalDir.dot(InitDir);
  double y = (InitDir - x * GoalDir).length();
  double angle_rad = atan2 (y,x);
  double angle_cur = angle_rad * (1.0 - cur_s);
  double proj = cos(angle_cur);
  return proj;
}

static double EdgeProjMagnitude(double cur_s,  const Vector3 & InitxDir, const Vector3 & GoalDir){
  double xProj = InitxDir.dot(GoalDir);
  return (1.0 - cur_s) * xProj;
}

std::vector<double> StagePlanningComputation( double sVal, Robot & SimRobotInner, 
                                              const std::vector<Vector3> EndEffecetorAxes,
                                              const std::vector<int> & SwingLinkChain,
                                              const SelfCollisionInfo & SelfCollisionInfoObj, 
                                              const CubicSplineInfo & CubicSplineInfoObj, 
                                              const SimPara & SimParaObj, bool & StagePlanningFlag){

  Vector3 GoalContactDirection = SimParaObj.getGoalContactDirection();
  Vector3 CurContactPos = CubicSplineInfoObj.s2Pos(sVal);
  std::vector<Vector3> StageIKVector3Vec = {CurContactPos, GoalContactDirection}; 

  int     SwingLinkInfoIndex = SimParaObj.getSwingLinkInfoIndex();

  double  EndEffectorProjx = EdgeProjMagnitude(sVal,  EndEffecetorAxes[0], GoalContactDirection);
  double  EndEffectorProjy = EdgeProjMagnitude(sVal,  EndEffecetorAxes[1], GoalContactDirection);
  double  EndEffectorProjz = ProjectionLength(sVal,   EndEffecetorAxes[2], GoalContactDirection);

  std::vector<double> StageIKVec = {sVal, EndEffectorProjx, EndEffectorProjy, EndEffectorProjz}; 
  
  std::string ConfigPath = "./";
  std::string OptConfigFile = "Before.config";
  RobotConfigWriter(SimRobotInner.q, ConfigPath, OptConfigFile);

  std::vector<double> UpdatedConfig = StageIKOptimization(SimRobotInner, SwingLinkInfoIndex,
                                                          SwingLinkChain, 
                                                          SelfCollisionInfoObj,  
                                                          StageIKVector3Vec, 
                                                          StageIKVec, StagePlanningFlag);
  OptConfigFile = "After.config";
  RobotConfigWriter(UpdatedConfig, ConfigPath, OptConfigFile);

  return UpdatedConfig;
}
