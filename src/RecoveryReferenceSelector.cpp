#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

 // Select Estimated Failure Metric
 int RecoveryReferenceSelector(const std::vector<double> & EstFailureMetricVec, const std::vector<RecoveryReferenceInfoObj> & RecoveryReferenceInfoObjVec, int & LastEndEffectorIndex, const SimPara & SimParaObj){
   int RecoveryReferenceIndex = -1;
   if(EstFailureMetricVec.size()){
   int ObjIndex;
   if(LastEndEffectorIndex == -1){
    //  This indicates that there is no previous protective contact.
     ObjIndex = std::distance(EstFailureMetricVec.begin(), std::max_element(EstFailureMetricVec.begin(), EstFailureMetricVec.end()));
     RecoveryReferenceInfoObj = RecoveryReferenceInfoObjVec[ObjIndex];
     LastEndEffectorIndex = RecoveryReferenceInfoObj.getSwingLinkInfoIndex();
   } else {
     ObjIndex = std::distance(EstFailureMetricVec.begin(), std::max_element(EstFailureMetricVec.begin(), EstFailureMetricVec.end()));
     RecoveryReferenceInfoObj = RecoveryReferenceInfoObjVec[ObjIndex];
     if(RecoveryReferenceInfoObj.getSwingLinkInfoIndex() == LastEndEffectorIndex){
       EstFailureMetricVec[ObjIndex] = -1.0;
       ObjIndex = std::distance(EstFailureMetricVec.begin(), std::max_element(EstFailureMetricVec.begin(), EstFailureMetricVec.end()));
       RecoveryReferenceInfoObj = RecoveryReferenceInfoObjVec[ObjIndex];
       LastEndEffectorIndex = RecoveryReferenceInfoObj.getSwingLinkInfoIndex();
     } else
        LastEndEffectorIndex = RecoveryReferenceInfoObj.getSwingLinkInfoIndex();
   }
   PlanTimeRecorder(StagePlanningTime, SimParaObj.getCurrentCasePath());
   PlanningInfoFileAppender(SimParaObj.getPlanStageIndex(), EstFailureMetricVec.size()-1, SimParaObj.getCurrentCasePath(), SimParaObj.getSimTime());
 }
 return RecoveryReferenceIndex; 
}
 