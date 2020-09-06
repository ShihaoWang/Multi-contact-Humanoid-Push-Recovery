#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

LinearPath InitialSimulation(WorldSimulation & Sim, const SimPara & SimParaObj){
  const char *CtrlStateTrajStr_Name     = SimParaObj.CtrlStateTrajStr.c_str();
  const char *PlanStateTrajStr_Name     = SimParaObj.PlanStateTrajStr.c_str();
  LinearPath InitTraj;
  bool contacted=false;
  while(Sim.time <= SimParaObj.InitDuration){
    InitTraj.Append(Sim.time, Sim.world->robots[0]->q);
    std::printf("Plan Initial Simulation Time: %f\n", Sim.time);
    StateTrajAppender(CtrlStateTrajStr_Name,    Sim.time, Sim.world->robots[0]->q);
    StateTrajAppender(PlanStateTrajStr_Name,    Sim.time, Sim.world->robots[0]->q);
    Vector3 COMPos, COMVel;
    getCentroidalState(*Sim.world->robots[0], COMPos, COMVel);
    Vector3Appender(SimParaObj.CtrlPosTrajStr.c_str(), Sim.time, COMPos);
    Vector3Appender(SimParaObj.CtrlVelTrajStr.c_str(), Sim.time, COMVel);
    Sim.Advance(SimParaObj.TimeStep);
    Sim.UpdateModel();
  }
  std::printf("Plan Initial Simulation Done!\n");
  return InitTraj;
}

void PushImposer(WorldSimulation & Sim, double CurTime, const SimPara & SimParaObj, bool FailureFlag){
  if((CurTime<SimParaObj.PushDuration)&&(!FailureFlag)){
    int LinkIndex = 19;
    double ImpulseScale = 1.0 * CurTime/SimParaObj.PushDuration;
    Vector3 ImpulseForce = ImpulseScale * SimParaObj.ImpulseForceValue * SimParaObj.ImpulseForceDirection;
    dBodyAddForceAtRelPos(Sim.odesim.robot(0)->body(LinkIndex), ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, 0.0, 0.0, 0.0);
    PushInfoFileAppender(Sim.time, ImpulseForce.x, ImpulseForce.y, ImpulseForce.z, SimParaObj.CurrentCasePath);
  }
}