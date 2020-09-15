// This function is used to extensively simulate the result from the four failure metric
#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

int SimulationWithMethod(WorldSimulation & Sim, const std::vector<ContactStatusInfo> & InitContactInfo, SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj){
  /* Simulation parameters */
  double  DetectionWait   = SimParaObj.DetectionWait;
  double  DetectionCount  = DetectionWait;
  int     PlanStageIndex  = 0;
  bool    TouchDownFlag   = false;
  bool    CtrlFlag        = false;
  double  CtrlStartTime   = 0.0;

  /* Override the default controller with a RobotController */
  auto NewControllerPtr = std::make_shared<RobotController>(*Sim.world->robots[0]);
  Sim.SetController(0, NewControllerPtr);
  NewControllerPtr->SetPIDCommand(Sim.world->robots[0]->q);

  // Initial Simulation
  LinearPath InitStateTraj, CtrlStateTraj, PlanStateTraj;
  InitStateTraj = InitialSimulation(Sim, SimParaObj);
  CtrlStateTraj = InitStateTraj;
  PlanStateTraj = InitStateTraj;
  
  std::vector<double> qDes = Sim.world->robots[0]->q; 
  std::vector<double> qVis = Sim.world->robots[0]->q; 
  
  double InitTime = Sim.time;
  SimParaObj.FailureTime = 0.0;

  RecoveryReferenceInfo   RecoveryReferenceInfoObj;                            // Used for control reference generation.
  Robot                   SimRobot;
  bool                    FailureFlag = false;
  bool                    OverallFailureFlag = false;
  std::vector<ContactStatusInfo> curContactInfo =  InitContactInfo;
  while(Sim.time <= SimParaObj.TotalDuration + SimParaObj.InitDuration){
    SimRobot = *Sim.world->robots[0];
    qVis = SimRobot.q;
    double SimTime = Sim.time;
    if(!FailureFlag) PushImposer(Sim,  SimTime - InitTime, SimParaObj, FailureFlag);

    Vector3 COMPos, COMVel;
    getCentroidalState(SimRobot, COMPos, COMVel);
    std::vector<Vector3> ActContactPos = ActiveContactFinder(SimRobot, curContactInfo);
    std::vector<PIPInfo> PIPTotal = PIPGenerator(COMPos, COMVel, ActContactPos);
    ContactPolytopeWriter(ActContactPos, PIPTotal, SimParaObj);
    SelfCollisionInfoObj.SelfCollisionBoundingBoxesUpdate(SimRobot); double Dist; Vector3 DistGrad;
    
    if(!OverallFailureFlag){
      double FailureMetric = FailureMetricEval(PIPTotal);
      if(!CtrlFlag){
        if(DetectionCount>=DetectionWait){
          std::printf("Simulation Time: %f, and Failure Metric Value: %f\n", Sim.time, FailureMetric);
          if((FailureMetric < 0.0) && (!FailureFlag)) {
            FailureFlag = true;    
            if(SimParaObj.FailureTime * SimParaObj.FailureTime < 1e-12)
            SimParaObj.FailureTime = SimTime; 
          }
          if(FailureFlag){
            SimParaObj.setPlanStageIndex(PlanStageIndex);
            SimParaObj.setSimTime(SimTime);
            RecoveryReferenceInfoObj = RecoveryReferenceComputation(SimRobot, curContactInfo, SelfCollisionInfoObj, SimParaObj);
            if(RecoveryReferenceInfoObj.getReadyFlag()){
              CtrlFlag = true;
              CtrlStartTime = SimTime;
              PlanStageIndex++;
            }
          }
        } 
        else{
          DetectionCount+=SimParaObj.TimeStep;
          std::printf("Simulation Time: %f, and Failure Metric Value: %f\n", Sim.time, FailureMetric);
        }
       } 
       else{
        double InnerTime = SimTime - CtrlStartTime;
        // std::pair<Config, Config> ConfigPair = ConfigReferenceGene(SimRobot, InnerTime, RMObject, SelfLinkGeoObj, RecoveryReferenceInfoObj, SimParaObj);
        // qDes = ConfigPair.first;
        // qVis = ConfigPair.second;
        if (RecoveryReferenceInfoObj.getTouchDownFlag()){
          CtrlFlag = false;
          DetectionCount = 0.0;
          curContactInfo = RecoveryReferenceInfoObj.GoalContactStatus;
          FailureFlag = false;
        }
      }
    }
    OverallFailureFlag = FailureChecker(SimRobot);    
    Vector3Appender(SimParaObj.CtrlPosTrajStr.c_str(), Sim.time, COMPos);
    Vector3Appender(SimParaObj.CtrlVelTrajStr.c_str(), Sim.time, COMVel);
    NewControllerPtr->SetPIDCommand(Config(qDes));
    StateLogger(Sim, CtrlStateTraj, PlanStateTraj, qVis, SimParaObj);
    Sim.Advance(SimParaObj.TimeStep);
    Sim.UpdateModel();
  }
  if(!FailureChecker(SimRobot)) return 1;
  return 0;
}
