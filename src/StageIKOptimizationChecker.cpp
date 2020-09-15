#include "CommonHeader.h"

extern std::vector<LinkInfo>   LinkInfoObj;
extern SDFInfo                 SDFInfoObj;

static double SelfCollisionTol = 1e-2;
static double ContactReachTol  = 0.05;  // 5cm

bool StageIKOptimizationChecker(const Robot & SimRobotObj, int SwingLinkInfoIndex, const std::vector<int> & SwingLinkChain, const SelfCollisionInfo & SelfCollisionInfoObj, const Vector3 & GoalPos, const std::vector<double> & OptConfig){

  // Self-collision constraint numerical checker
  std::vector<double> SelfCollisionDistVec(SwingLinkChain.size()-3);
  for (int i = 0; i < SwingLinkChain.size()-3; i++)     // Due to the bounding box size of torso link
  {
    Box3D Box3DObj = SimRobotObj.geometry[SwingLinkChain[i]]->GetBB();
    std::vector<Vector3> BoxVerticesVec = BoxVertices(Box3DObj);
    std::vector<double> DistVec(BoxVerticesVec.size());
    for (int j = 0; j < BoxVerticesVec.size(); j++)
      DistVec[j] = SelfCollisionInfoObj.getSelfCollisionDist(SwingLinkInfoIndex, BoxVerticesVec[j]);
    SelfCollisionDistVec[i] = *std::min_element(DistVec.begin(), DistVec.end());
  }
  double SelfCollisionDistTol = *std::min_element(SelfCollisionDistVec.begin(), SelfCollisionDistVec.end());

  bool OptFlag = true;
  if(SelfCollisionDistTol<-SelfCollisionTol){
      std::printf("StageIKOptimazation Failure due to Self-collision for Link %d with Depth %f! \n", 
                  LinkInfoObj[SwingLinkInfoIndex].LinkIndex, SelfCollisionDistTol);
      OptFlag = false;
  }

  // Vector3 EndEffectorAvgPos;
  // SimRobotObj.GetWorldPosition(   LinkInfoObj[SwingLinkInfoIndex].AvgLocalContact, 
  //                                 LinkInfoObj[SwingLinkInfoIndex].LinkIndex, 
  //                                 EndEffectorAvgPos);
  // Vector3 AvgDiff = EndEffectorAvgPos - GoalPos;
  // double DistTest = AvgDiff.normSquared();
  // if(DistTest>(ContactReachTol * ContactReachTol)){
  //   std::printf("StageIKOptimazation Failure due to Goal Contact Non-reachability for Link %d for %f! \n", 
  //               LinkInfoObj[SwingLinkInfoIndex].LinkIndex, DistTest);
  //   OptFlag = false;
  // }
  return OptFlag;
}
