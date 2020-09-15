#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

 // Select Estimated Failure Metric
RecoveryReferenceInfo RecoveryReferenceSelector(const double & StagePlanningTime, const std::vector<double> & EstFailureMetricVecInner, const std::vector<RecoveryReferenceInfo> & RecoveryReferenceInfoObjVec, int & LastEndEffectorIndex, const SimPara & SimParaObj){
  int ObjIndex;
  std::vector<double> EstFailureMetricVec = EstFailureMetricVecInner;
  RecoveryReferenceInfo RecoveryReferenceInfoObj;
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
        } else LastEndEffectorIndex = RecoveryReferenceInfoObj.getSwingLinkInfoIndex();
    }
    PlanTimeRecorder(StagePlanningTime, SimParaObj.getCurrentCasePath());
    PlanningInfoFileAppender(SimParaObj.getPlanStageIndex(), EstFailureMetricVec.size()-1, SimParaObj.getCurrentCasePath(), SimParaObj.getSimTime());
  return RecoveryReferenceInfoObj;
}
 