#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

void InitialSimulation(WorldSimulation & Sim, const SimPara & SimParaObj, LinearPath & CtrlStateTraj, LinearPath & PlanStateTraj){
  const char *CtrlStateTrajStr_Name     = SimParaObj.CtrlStateTrajStr.c_str();
  const char *PlanStateTrajStr_Name     = SimParaObj.PlanStateTrajStr.c_str();
  // const char *CtrlCFTrajStr_Name        = SimParaObj.CtrlCFTrajStr.c_str();
  // const char *FailureCFTrajStr_Name     = SimParaObj.FailureCFTrajStr.c_str();
  LinearPath InitTraj;
  Vector3 F1, F2, F3, F4;
  bool contacted=false;
  while(Sim.time <= SimParaObj.InitDuration){
    InitTraj.Append(Sim.time, Sim.world->robots[0]->q);
    std::printf("Plan Initial Simulation Time: %f\n", Sim.time);
    StateTrajAppender(CtrlStateTrajStr_Name,    Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(PlanStateTrajStr_Name,    Sim.time, Sim.world->robots[0]->q);
    Vector3 COMPos, COMVel;
    getCentroidalState(*Sim.world->robots[0], COMPos, COMVel);
    ContactForceAppender(SimParaObj.CtrlVelTrajStr.c_str(), Sim.time, COMVel);
    Sim.Advance(SimParaObj.TimeStep);
    Sim.UpdateModel();

  }
  CtrlStateTraj = InitTraj;
  PlanStateTraj = InitTraj;  
  std::printf("Plan Initial Simulation Done!\n");
  return;
}

void PushImposer(WorldSimulation & Sim, const double & CurTime, const SimPara & SimParaObj, const bool & FailureFlag){
  if((CurTime<SimParaObj.PushDuration)&&(!FailureFlag)){
    int LinkIndex = 19;
    double ImpulseScale = 1.0 * CurTime/SimParaObj.PushDuration;
    Vector3 ImpulseForce = ImpulseScale * SimParaObj.ImpulseForceMax;
    dBodyAddForceAtRelPos(Sim.odesim.robot(0)->body(LinkIndex), ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, 0.0, 0.0, 0.0);
    PushInfoFileAppender(Sim.time, ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, SimParaObj.CurrentCasePath);
  }
}

static double PlanSwingContactDistCal(Robot SimRobotObj, int SwingLinkInfoIndex, Config PlanConfig){
  SimRobotObj.UpdateConfig(PlanConfig);
  std::vector<double> EndEffectorSDVec;
  for (Vector3 & LocalContact : NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LocalContacts) {
    Vector3 SwingLinkContactPos;
    SimRobotObj.GetWorldPosition(LocalContact, NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex, SwingLinkContactPos);
    double CurrentDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SwingLinkContactPos);
    EndEffectorSDVec.push_back(CurrentDist);
  }
  double SwingContactDist = *min_element(EndEffectorSDVec.begin(), EndEffectorSDVec.end());
  return SwingContactDist;
}

std::pair<Config, Config> ConfigReferenceGene(const Robot & SimRobotObj,  double & InnerTime,
                                              ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj,
                                              ControlReferenceInfo & ControlReference, SimPara & SimParaObj){
  // This function generates robot's reference configuration at each time.
  std::printf("InnerTime: %f\n", InnerTime);
  double TouchDownTol  = 0.01;                      //  1 cm as a Touch Down Terminal Tolerance.
  std::vector<double> qDes;
  int SwingLinkInfoIndex = ControlReference.getSwingLinkInfoIndex();
  std::vector<double> EndEffectorSDVec;
  for (Vector3 & LocalContact : NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LocalContacts) {
    Vector3 SwingLinkContactPos;
    SimRobotObj.GetWorldPosition(LocalContact, NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex, SwingLinkContactPos);
    double CurrentDist = NonlinearOptimizerInfo::SDFInfo.SignedDistance(SwingLinkContactPos);
    EndEffectorSDVec.push_back(CurrentDist);
  }
  double CtrlSwingContactDist = *min_element(EndEffectorSDVec.begin(), EndEffectorSDVec.end());

  Config qDesConfig, qVisConfig;
  // Pure Open-loop Controller
  ControlReference.PlannedConfigTraj.Eval(InnerTime, qDesConfig); 
  qVisConfig = SimRobotObj.q;
  std::vector<int> SwingLinkChain = RMObject.EndEffectorLink2Pivotal[SwingLinkInfoIndex];
  for (int i = 0; i < SwingLinkChain.size(); i++)
    qVisConfig[SwingLinkChain[i]] = qDesConfig[SwingLinkChain[i]];

  double PlanSwingContactDist = PlanSwingContactDistCal(SimRobotObj, SwingLinkInfoIndex, qVisConfig);

  double SwingContactDist = min(CtrlSwingContactDist, PlanSwingContactDist);

  // // IK Path Tracking Controller
  // Vector GoalPosVec; 
  // ControlReference.EndEffectorTraj.Eval(InnerTime + SimParaObj.TimeStep, GoalPosVec);
  // double sCur = (InnerTime + SimParaObj.TimeStep)/ControlReference.PlannedConfigTraj.EndTime();
  // double EndEffectorProjx = EdgeProjMagnitude(sCur, SimParaObj.EndEffectorInitxDir, SimParaObj.DirectionGoal);
  // double EndEffectorProjy = EdgeProjMagnitude(sCur, SimParaObj.EndEffectorInityDir, SimParaObj.DirectionGoal);
  // bool IKFlag = true;
  // Vector3 GoalPos(GoalPosVec);
  // std::vector<double> qVisConfigIK = IKConfigOptimazation(SimRobotObj, RMObject, SelfLinkGeoObj, GoalPos, SimParaObj.DirectionGoal, SwingLinkInfoIndex, sCur, EndEffectorProjx, EndEffectorProjy, IKFlag);
  // if(IKFlag){
  //   qVisConfig = qVisConfigIK;
  //   std::vector<int> SwingLinkChain = RMObject.EndEffectorLink2Pivotal[SwingLinkInfoIndex];
  //   for (int i = 0; i < SwingLinkChain.size(); i++){
  //     qDesConfig[SwingLinkChain[i]] = qVisConfig[SwingLinkChain[i]];
  //   }
  // }

  // // Cartesian Controller 
  // qDes = CartesianController(SimRobotObj, ControlReference, InnerTime, SimParaObj.TimeStep);

  double TimeRatio = 0.5;

  if(!ControlReference.getTouchDownFlag()){
    for (int i = 0; i < SimRobotObj.q.size(); i++)
        qDes.push_back(qDesConfig[i]);
  } else qDes = ControlReference.getTouchDownConfig();

  if(!ControlReference.getTouchDownFlag()){
    if(ControlReference.ControlReferenceType==1){
      // Contact Addition Case
      // if(InnerTime>=ControlReference.TimeTraj.back()){
      //   ControlReference.setTouchDownFlag(true);
      //   ControlReference.setTouchDownConfig(qDes);
      // }
      if(SwingContactDist<TouchDownTol){
        ControlReference.setTouchDownFlag(true);
        ControlReference.setTouchDownConfig(qDes);
      }
    }
    else{
      // Contact Modification Case
      if(InnerTime>TimeRatio * ControlReference.TimeTraj.back()){
        if(SwingContactDist<TouchDownTol){
          ControlReference.setTouchDownFlag(true);
          ControlReference.setTouchDownConfig(qDes);
        }
      }
    }
  }
  return std::make_pair(Config(qDesConfig), Config(qVisConfig));
}

int FailureTest(WorldSimulation & Sim, const std::vector<ContactStatusInfo> & InitContactInfo, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj, SimPara & SimParaObj){
    /* Simulation parameters */
  Config  InitConfig      = Sim.world->robots[0]->q;
  auto NewControllerPtr = std::make_shared<PolynomialPathController>(*Sim.world->robots[0]);
  Sim.SetController(0, NewControllerPtr);
  NewControllerPtr->SetConstant(InitConfig);
  // Initial Simulation
  const char *FailureStateTrajStr_Name  = SimParaObj.FailureStateTrajStr.c_str();
  while(Sim.time <= SimParaObj.InitDuration){
    std::printf("Failure Initial Simulation Time: %f\n", Sim.time);
    StateTrajAppender(FailureStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
    Vector3 COMPos, COMVel;
    getCentroidalState(*Sim.world->robots[0], COMPos, COMVel);
    StateTrajAppender(SimParaObj.FailureStateTrajStr.c_str(),    Sim.time, Sim.world->robots[0]->q);
    ContactForceAppender(SimParaObj.FailureVelTrajStr.c_str(), Sim.time, COMVel);
    Sim.Advance(SimParaObj.TimeStep);
    Sim.UpdateModel();
  }
  std::printf("Failure Initial Simulation Done!\n");

  bool FailureFlag;
  double TotalTime = SimParaObj.InitDuration + SimParaObj.TotalDuration;
  while(Sim.time <= TotalTime){
    double SimTime = Sim.time;
    if(SimTime<=SimParaObj.FailureTime) PushImposer(Sim,  SimTime - SimParaObj.InitDuration, SimParaObj, FailureFlag);
    Vector3 COMPos, COMVel;
    getCentroidalState(*Sim.world->robots[0], COMPos, COMVel);
    StateTrajAppender(SimParaObj.FailureStateTrajStr.c_str(), Sim.time, Sim.world->robots[0]->q);
    ContactForceAppender(SimParaObj.FailureVelTrajStr.c_str(), Sim.time, COMVel);
    Sim.Advance(SimParaObj.TimeStep);
    Sim.UpdateModel();
    std::printf("Failure Initial Simulation Time: %f\n", SimTime);
  }
  if(FailureChecker(*Sim.world->robots[0], RMObject)) return 1;
  return 0;
}