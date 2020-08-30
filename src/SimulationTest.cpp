// This function is used to extensively simulate the result from the four failure metric
#include "RobotInfo.h"
#include "CommonHeader.h"
#include <ode/ode.h>
#include "Control/PathController.h"
#include "Control/JointTrackingController.h"
#include "NonlinearOptimizerInfo.h"

int SimulationTest(WorldSimulation & Sim, const std::vector<ContactStatusInfo> & InitContactInfo, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj, SimPara & SimParaObj){
  /* Simulation parameters */
  Config  InitConfig      = Sim.world->robots[0]->q;
  int     DOF             = InitConfig.size();
  double  DetectionWait   = SimParaObj.DetectionWait;
  double  DetectionCount  = DetectionWait;
  int     PlanStageIndex  = 0;
  double  RHPDuration     = 0.25;                 // Duration for RHP executation until next planning
  double  RHPCounter      = RHPDuration;
  bool    RHPFlag         = false;                // True means RHP planning is working.
  bool    TouchDownFlag   = false;
  bool    CtrlFlag        = false;
  double  CtrlStartTime   = 0.0;

  /* Override the default controller with a PolynomialPathController */
  auto NewControllerPtr = std::make_shared<PolynomialPathController>(*Sim.world->robots[0]);
  Sim.SetController(0, NewControllerPtr);
  NewControllerPtr->SetConstant(InitConfig);

  // Initial Simulation
  LinearPath CtrlStateTraj, PlanStateTraj;
  InitialSimulation(Sim, SimParaObj, CtrlStateTraj, PlanStateTraj);
  std::vector<double> qDes = InitConfig;               // This is commanded robot configuration to the controller.
  double InitTime = Sim.time;
  SimParaObj.FailureTime = 0.0;

  ControlReferenceInfo  ControlReferenceObj;                            // Used for control reference generation.
  Robot                 SimRobot;
  bool                  FailureFlag = false;
  bool                  OverallFailureFlag = false;
  std::vector<ContactStatusInfo> curContactInfo =  InitContactInfo;
  while(Sim.time <= SimParaObj.TotalDuration + SimParaObj.InitDuration){
    SimRobot = *Sim.world->robots[0];
    double SimTime = Sim.time;
    if(!FailureFlag) PushImposer(Sim,  SimTime - InitTime, SimParaObj, FailureFlag);

    Vector3 COMPos, COMVel;
    getCentroidalState(SimRobot, COMPos, COMVel);
    std::vector<Vector3> ActContactPos = ActiveContactFinder(SimRobot, curContactInfo);
    std::vector<PIPInfo> PIPTotal = PIPGenerator(ActContactPos, COMPos, COMVel);
    ContactPolytopeWriter(ActContactPos, PIPTotal, SimParaObj);
    SelfLinkGeoObj.LinkBBsUpdate(SimRobot);

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
            ControlReferenceObj = ControlReferenceGene(SimRobot, curContactInfo, RMObject, SelfLinkGeoObj, SimParaObj);
            if(ControlReferenceObj.getReadyFlag()){
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
        qDes = ConfigReferenceGene(SimRobot, InnerTime, RMObject, SelfLinkGeoObj, ControlReferenceObj, SimParaObj);
        if (ControlReferenceObj.getTouchDownFlag()){
          CtrlFlag = false;
          DetectionCount = 0.0;
          curContactInfo = ControlReferenceObj.GoalContactStatus;
          FailureFlag = false;
        }
      }
    }
    OverallFailureFlag = FailureChecker(SimRobot, RMObject);    
    ContactForceAppender(SimParaObj.CtrlVelTrajStr.c_str(), Sim.time, COMVel);
    NewControllerPtr->SetConstant(Config(qDes));
    StateLogger(Sim, CtrlStateTraj, PlanStateTraj, qDes, SimParaObj);
    Sim.Advance(SimParaObj.TimeStep);
    Sim.UpdateModel();
  }
  int PushRecovSuccFlag = 0;
  if(!FailureChecker(SimRobot, RMObject)) return 1;
  return 0;
}
