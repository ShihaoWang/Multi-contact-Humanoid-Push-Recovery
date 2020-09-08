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

   RecoveryReferenceInfo RecoveryReferenceInfoObj_ = RecoveryReferenceComputationInner(SimRobot, TipOverPIP, SelfCollisionInfoObj, ContactFormObj, SimParaObj, ForwardTimeSeed);
   
   double planning_time = (std::clock() - StartTime)/(double)CLOCKS_PER_SEC;

   StagePlanningTime+=planning_time;
   StartTime = std::clock();
   
   RecoveryReferenceInfoObj_.setSwingLinkInfoIndex(ContactFormObj.SwingLinkInfoIndex);
   RecoveryReferenceInfoObj_.setControlReferenceType(ContactFormObj.ContactType);

   if(RecoveryReferenceInfoObj_.getReadyFlag()){
     RecoveryReferenceInfoObjVec.push_back(RecoveryReferenceInfoObj_);
     EstFailureMetricVec.push_back(RecoveryReferenceInfoObj_.getFailureMetric());
     SimParaObj.DataRecorderObj.UpdateWithRecoveryReferenceInfo(RecoveryReferenceInfoObj_.PlannedConfigTraj, 
                                                                RecoveryReferenceInfoObj_.EndEffectorTraj, 
                                                                RecoveryReferenceInfoObj_.getFailureMetric());
     SimParaObj.DataRecorderObj.Write2File(SimParaObj.getCurrentCasePath());
     PlanEndEffectorIndex++;
   }
 }
 return RecoveryReferenceInfoObj;
}
