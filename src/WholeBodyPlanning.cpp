#include "CommonHeader.h"
#include "RobotInfo.h"
extern ReachabilityMap         ReachabilityMapObj;

RecoveryReferenceInfo WholeBodyPlanning(  const Robot & SimRobotInner, const InvertedPendulumInfo & InvertedPendulumInner, 
                                          const SelfCollisionInfo & SelfCollisionInfoObjInner, 
                                          const CubicSplineInfo & CubicSplineInfoObj, const SimPara & SimParaObj){

  Robot                 SimRobot              =   SimRobotInner;   
  InvertedPendulumInfo  InvertedPendulumObj   =   InvertedPendulumInner;
  SelfCollisionInfo     SelfCollisionInfoObj  =   SelfCollisionInfoObjInner; 

  RecoveryReferenceInfo RecoveryReferenceInfoObj;

  int SwingLinkInfoIndex = SimParaObj.getSwingLinkInfoIndex();
  std::vector<int> SwingLinkChain = SwingLinkChainSelector(ReachabilityMapObj, SwingLinkInfoIndex, SimParaObj.getOneHandAlreadyFlag());

  // Vector3 EndEffectorInitxDir, EndEffectorInityDir, EndEffectorInitzDir;   // Eventually these two directions should be orthgonal to goal direction.
  std::vector<Vector3> EndEffecetorAxes = getEndEffectorXYZAxes(SimRobotInner, SwingLinkInfoIndex);

  std::vector<double>               TimeTraj;
  std::vector<Config>               WholeBodyConfigTraj;
  std::vector<Config>               WholeBodyVelocityTraj;
  std::vector<Vector3>              EndEffectorPosTraj;
  // std::vector<Vector3>              EndEffectorVelocityTraj;
  // std::vector<Vector3>              EndEffectorAngularVelocityTraj;
  std::vector<QuaternionRotation>   OrientationQuatTraj;

  double CurTime                    = 0.0;
  Config CurConfig                  = SimRobot.q;
  Config CurVelocity                = SimRobot.dq;
  Vector3 CurEndEffectorPos         = SimParaObj.getInitContactPos();
  // Vector3 CurEndEffectorVel         = getEndEffectorTranslationalVelocity(SimRobot, SwingLinkInfoIndex, CurVelocity);
  // Vector3 CurEndEffectorAngVel      = getEndEffectorAngularVelocity(SimRobot, SwingLinkInfoIndex, CurVelocity);
  QuaternionRotation CurEndEffectorQuaternion = getEndEffectorQuaternion(SimRobot, SwingLinkInfoIndex);

  TimeTraj.push_back(CurTime);
  WholeBodyConfigTraj.push_back(CurConfig);
  WholeBodyVelocityTraj.push_back(CurVelocity);
  EndEffectorPosTraj.push_back(CurEndEffectorPos);
  // EndEffectorVelocityTraj.push_back(CurEndEffectorVel);
  // EndEffectorAngularVelocityTraj.push_back(CurEndEffectorAngVel);
  OrientationQuatTraj.push_back(CurEndEffectorQuaternion);

  double sBoundary      = SimParaObj.PhaseRatio;
  double ReductionRatio = SimParaObj.ReductionRatio;
  
  double sVal             = 0.0;
  double sUnit            = 0.2;

  bool TouchDownFlag      = false;                // This is a last stage flag.
  bool PenetrationFlag    = false;                // This is a failure flag.
  bool StagePlanningFlag  = false;
  bool FeasibleFlag       = false;
  int  StageIndex         = 0;

  while ((sVal<1.0) && (!TouchDownFlag) && (!PenetrationFlag)){
    SimRobot.UpdateConfig(CurConfig);
    SimRobot.UpdateGeometry();
    SelfCollisionInfoObj.SelfCollisionBoundingBoxesUpdate(SimRobot);

    sVal+=sUnit;

    std::vector<double> UpdatedConfig = StagePlanningComputation( sVal, SimRobot, 
                                                                  EndEffecetorAxes,
                                                                  SwingLinkChain,
                                                                  SelfCollisionInfoObj, 
                                                                  CubicSplineInfoObj, 
                                                                  SimParaObj, StagePlanningFlag);   
    if(!StagePlanningFlag) FeasibleFlag = false;
    std::vector<double> NextConfig    =   UpdatedConfig;
    std::vector<double> NextVelocity  =   WholeBodyVelocityTraj.back();
    double StageTime = StageTimeComputation(SimRobot, CurConfig, CurVelocity, Config(UpdatedConfig), SwingLinkChain, 
                                            sVal, SimParaObj, NextConfig, NextVelocity);
    
    SimRobot.UpdateConfig(Config(NextConfig));
    NextConfig = WholeBodyDynamicsIntegrator(SimRobot, SwingLinkInfoIndex, InvertedPendulumObj, StageTime, TouchDownFlag, PenetrationFlag);
    SimRobot.UpdateConfig(Config(NextConfig));
    if(((TouchDownFlag) && (!PenetrationFlag))||(sVal>= 1.0)){
      bool TouchDownOptFlag;  
      std::vector<double> TouchDownConfig = TouchDownConfigOptimazation(SimRobot, SwingLinkInfoIndex, 
                                                                        SwingLinkChain, 
                                                                        SelfCollisionInfoObj,  
                                                                        SimParaObj, TouchDownOptFlag);

      FeasibleFlag = TouchDownOptFlag;
      NextConfig = TouchDownConfig;
      SimRobot.UpdateConfig(Config(NextConfig));
    }

    std::string ConfigPath = "./";
    std::string OptConfigFile = "StageConfig" + to_string(StageIndex) + ".config";
    RobotConfigWriter(UpdatedConfig, ConfigPath, OptConfigFile);
    OptConfigFile = "NextConfig" + to_string(StageIndex) + ".config";
    RobotConfigWriter(NextConfig, ConfigPath, OptConfigFile);

    CurTime+=StageTime;
    CurConfig = NextConfig;
    CurVelocity = Config(NextVelocity);
    CurEndEffectorQuaternion = getEndEffectorQuaternion(SimRobot, SwingLinkInfoIndex);

    TimeTraj.push_back(CurTime);
    WholeBodyConfigAppender(WholeBodyConfigTraj, Config(NextConfig));
    WholeBodyVelocityTraj.push_back(Config(NextVelocity));
    EndEffectorPosTraj.push_back(CubicSplineInfoObj.s2Pos(sVal));
    OrientationQuatTraj.push_back(CurEndEffectorQuaternion);

    SelfCollisionInfoObj.SelfCollisionBoundingBoxesUpdate(SimRobot);
    StageIndex++;
  }

  // LinearPath WholeBodyConfigTrajPath(TimeTraj, WholeBodyConfigTraj);
  // std::ofstream WholeBodyConfigTrajFile;
  // const string  WholeBodyConfigTrajName = "WholeBodyConfigTraj.path";
  // WholeBodyConfigTrajFile.open(WholeBodyConfigTrajName.c_str());
  // WholeBodyConfigTrajPath.Save(WholeBodyConfigTrajFile);
  // WholeBodyConfigTrajFile.close();

  if(FeasibleFlag){
    std::vector<ContactStatusInfo> GoalContactInfo = SimParaObj.getFixedContactStatusInfo();
    for(int i = 0; i<GoalContactInfo[SwingLinkInfoIndex].LocalContactStatus.size(); i++)
      GoalContactInfo[SwingLinkInfoIndex].LocalContactStatus[i] = 1;
    
    RecoveryReferenceInfoObj.SetInitContactStatus(SimParaObj.FixedContactStatusInfo);
    RecoveryReferenceInfoObj.SetGoalContactStatus(GoalContactInfo);

    RecoveryReferenceInfoObj.TrajectoryUpdate(TimeTraj, WholeBodyConfigTraj, WholeBodyVelocityTraj, EndEffectorPosTraj);
    RecoveryReferenceInfoObj.OrientationQuat = OrientationQuatTraj;

    InvertedPendulumInfo InvertedPendulumFM = InvertedPendulumInner;
    bool TouchDownFlag, PenetrationFlag;
    SimRobot.UpdateConfig(WholeBodyConfigTraj.front());
    SimRobot.dq = WholeBodyVelocityTraj.front();

    // std::string ConfigPath = "./";
    // std::string OptConfigFile = "Front.config";
    // RobotConfigWriter(WholeBodyConfigTraj.front(), ConfigPath, OptConfigFile);

    Config EstConfig = WholeBodyDynamicsIntegrator(SimRobot, SwingLinkInfoIndex, InvertedPendulumFM, TimeTraj.back(), TouchDownFlag, PenetrationFlag);
    SimRobot.UpdateConfig(WholeBodyConfigTraj.back());
    SimRobot.dq = WholeBodyVelocityTraj.back();  
    double EstFailureMetric = EstimatedFailureMetric(SimRobot, GoalContactInfo, InvertedPendulumFM.COMPos, InvertedPendulumFM.COMVel);

    // OptConfigFile = "Back.config";
    // RobotConfigWriter(WholeBodyConfigTraj.back(), ConfigPath, OptConfigFile);

    RecoveryReferenceInfoObj.setFailureMetric(EstFailureMetric);
    RecoveryReferenceInfoObj.setOneHandAlreadyFlag(SimParaObj.getOneHandAlreadyFlag());
    RecoveryReferenceInfoObj.setReadyFlag(true);
  }
  return RecoveryReferenceInfoObj;
}