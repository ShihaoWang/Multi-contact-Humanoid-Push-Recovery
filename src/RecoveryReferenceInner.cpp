#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

extern std::vector<LinkInfo> LinkInfoObj;
extern SDFInfo SDFInfoObj;

RecoveryReferenceInfo RecoveryReferenceComputationInner(const Robot & SimRobot,                   const PIPInfo & TipOverPIPObj, 
                                                        SelfCollisionInfo & SelfCollisionInfoObj, const ContactForm & ContactFormObj, 
                                                        SimPara & SimParaObj,                     double ForwardTime){
  RecoveryReferenceInfo RecoveryReferenceInfoObj;
  int SwingLinkInfoIndex = SimParaObj.getSwingLinkInfoIndex();
  Vector3 InitContactPos;       // This is the position of the reference contact for robot's active end effector.
  SimRobot.GetWorldPosition(LinkInfoObj[SwingLinkInfoIndex].AvgLocalContact, LinkInfoObj[SwingLinkInfoIndex].LinkIndex, InitContactPos);
  SimParaObj.setInitContactPos(InitContactPos);
  
  std::vector<Vector3> OptimalContact = OptimalContactSearcher(SimRobot, TipOverPIPObj, ContactFormObj, SimParaObj, ForwardTime);
  if(!OptimalContact.size()) return RecoveryReferenceInfoObj;
  SimParaObj.setFixedContactStatusInfo(ContactFormObj.FixedContactStatusInfo);

  Vector3 COMPos, COMVel;
  getCentroidalState(SimRobot, COMPos, COMVel);
  InvertedPendulumInfo InvertedPendulumObj( TipOverPIPObj.L, TipOverPIPObj.g,
                                            TipOverPIPObj.theta, TipOverPIPObj.thetadot,
                                            COMPos, COMVel);
  InvertedPendulumObj.setEdges(TipOverPIPObj.edge_a, TipOverPIPObj.edge_b);
  for (int i = 0; i < OptimalContact.size(); i++) {
    Robot SimRobotInner = SimRobot;
    SimParaObj.setGoalContactPos(OptimalContact[i]);
    SimParaObj.setGoalContactDirection(SDFInfoObj.SignedDistanceNormal(OptimalContact[i])); 

    CubicSplineInfo CubicSplineInfoObj = EndEffectorPathComputation(SimRobotInner, SelfCollisionInfoObj, SimParaObj);
    
    if(CubicSplineInfoObj.getReadyFlag()){
      RecoveryReferenceInfoObj = WholeBodyPlanning(SimRobot, InvertedPendulumObj, SelfCollisionInfoObj, CubicSplineInfoObj, SimParaObj);
      if(RecoveryReferenceInfoObj.getReadyFlag()) break;
      }
    }
    return RecoveryReferenceInfoObj;
  }