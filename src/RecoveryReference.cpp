#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <ctime>
#include <queue>
#include <algorithm>    // std::min

extern std::vector<LinkInfo>   LinkInfoObj;
// SDFInfo                 SDFInfoObj;
// ReachabilityMap         ReachabilityMapObj;
// AnyCollisionGeometry3D  TerrColGeomObj;

static int LastEndEffectorIndex = -1;

RecoveryReferenceInfo RecoveryReferenceComputation( const Robot & SimRobot,
                                                    const std::vector<ContactStatusInfo> & curRobotContactInfo,
                                                    SelfCollisionInfo & SelfCollisionInfoObj,
                                                    SimPara & SimParaObj){
 Vector3 COMPos, COMVel;
 getCentroidalState(SimRobot, COMPos, COMVel);

 RecoveryReferenceInfo RecoveryReferenceInfoObj;

 std::vector<ContactForm> ContactFormVec = getCandidateContactStatus(SimRobot, curRobotContactInfo);

 std::vector<RecoveryReferenceInfo> RecoveryReferenceInfoObjVec;
 std::vector<double> EstFailureMetricVec;

 int PlanEndEffectorIndex = 0;
 std::clock_t StartTime = std::clock();
 double StagePlanningTime = 0.0;

 for (auto & ContactFormObj : ContactFormVec) {
   
   std::vector<Vector3> ActContactPos = ActiveContactFinder(SimRobot, ContactFormObj.FixedContactStatusInfo);
   
   bool TipOverPIPValidFlag;
   PIPInfo TipOverPIP = TipOverPIPGenerator(COMPos, COMVel, ActContactPos, TipOverPIPValidFlag);
   if(!TipOverPIPValidFlag) continue;
   
   SimParaObj.setSwingLinkInfoIndex(ContactFormObj.SwingLinkInfoIndex);
   SimParaObj.setPlanEndEffectorIndex(PlanEndEffectorIndex);
   SimParaObj.setOneHandAlreadyFlag(OneHandAlreadyChecker(ContactFormObj));
   
   SimParaObj.DataRecorderObj.setPlanStageIndexNLinkNo(SimParaObj.getPlanStageIndex(), PlanEndEffectorIndex);

   double ForwardTimeSeed = SimParaObj.ForwardDurationSeed;
   double ForwardTimeAct = -1.0;
   RecoveryReferenceInfo RecoveryReferenceInfoObjInner;
   int MaxIter = 10;
   double ForwardTime = ForwardTimeSeed;
   double ForwardTimeTol = 0.01;
   for (int Iter = 0; Iter < MaxIter; Iter++){
     RecoveryReferenceInfoObjInner = RecoveryReferenceComputationInner(SimRobot, TipOverPIP, SelfCollisionInfoObj, ContactFormObj, SimParaObj, ForwardTime);
     ForwardTimeAct = RecoveryReferenceInfoObjInner.TimeTraj.back();
     double ForwardTimeDiff = ForwardTimeAct - ForwardTime;
     if(ForwardTimeDiff * ForwardTimeDiff< ForwardTimeTol * ForwardTimeTol){
       break;
     } else {
       ForwardTime = ForwardTimeAct;
     }
   }
   double planning_time = (std::clock() - StartTime)/(double)CLOCKS_PER_SEC;
   std::printf("planning_time: %f\n", planning_time);
   StagePlanningTime+=planning_time;
   StartTime = std::clock();
   
   RecoveryReferenceInfoObjInner.setSwingLinkInfoIndex(ContactFormObj.SwingLinkInfoIndex);
   RecoveryReferenceInfoObjInner.setControlReferenceType(ContactFormObj.ContactType);

   if(RecoveryReferenceInfoObjInner.getReadyFlag()){
     RecoveryReferenceInfoObjVec.push_back(RecoveryReferenceInfoObjInner);
     EstFailureMetricVec.push_back(RecoveryReferenceInfoObjInner.getFailureMetric());
     SimParaObj.DataRecorderObj.UpdateWithRecoveryReferenceInfo(RecoveryReferenceInfoObjInner.PlannedConfigTraj, 
                                                                RecoveryReferenceInfoObjInner.EndEffectorTraj, 
                                                                RecoveryReferenceInfoObjInner.getFailureMetric());
     SimParaObj.DataRecorderObj.Write2File(SimParaObj.getCurrentCasePath());
     PlanEndEffectorIndex++;
   }
 }
 if(EstFailureMetricVec.size()){
        RecoveryReferenceInfoObj = RecoveryReferenceSelector( StagePlanningTime, 
                                                              EstFailureMetricVec, 
                                                              RecoveryReferenceInfoObjVec, 
                                                              LastEndEffectorIndex,
                                                              SimParaObj);
 }
 return RecoveryReferenceInfoObj;
}
