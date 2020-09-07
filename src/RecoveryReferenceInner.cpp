#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

extern std::vector<LinkInfo> LinkInfoObj;

RecoveryReferenceInfo RecoveryReferenceComputationInner(const Robot & SimRobot,                   const PIPInfo & TipOverPIPObj, 
                                                        SelfCollisionInfo & SelfCollisionInfoObj, const ContactForm & ContactFormObj, 
                                                        SimPara & SimParaObj){
  RecoveryReferenceInfo ControlReferenceObj;
  int SwingLinkInfoIndex = SimParaObj.getSwingLinkInfoIndex();
  Vector3 InitContactPos;       // This is the position of the reference contact for robot's active end effector.
  SimRobot.GetWorldPosition(LinkInfoObj[SwingLinkInfoIndex].AvgLocalContact, LinkInfoObj[SwingLinkInfoIndex].LinkIndex, InitContactPos);
  SimParaObj.setInitContactPos(InitContactPos);
  
  std::vector<Vector3> OptimalContact = OptimalContactSearcher(SimRobot, TipOverPIPObj, ContactFormObj, SimParaObj);
  if(!OptimalContact.size()) return ControlReferenceObj;
  SimParaObj.setFixedContactStatusInfo(ContactFormObj.FixedContactStatusInfo);

  Vector3 COMPos, COMVel;
  getCentroidalState(SimRobot, COMPos, COMVel);
  InvertedPendulumInfo InvertedPendulumObj( TipOverPIPObj.L, TipOverPIPObj.g,
                                            TipOverPIPObj.theta, TipOverPIPObj.thetadot,
                                            COMPos, COMVel);
  InvertedPendulumObj.setEdges(TipOverPIPObj.edge_a, TipOverPIPObj.edge_b);
  // for (int i = 0; i < OptimalContact.size(); i++) {
  //   Robot SimRobotInner = SimRobot;
  //   SimParaObj.setContactGoal(OptimalContact[i]);
  //   SimParaObj.setTransPathFeasiFlag(false);
  //   CubicSplineInfo CubicSplineInfoObj = TransientPathGene(SimRobotInner, SelfLinkGeoObj, SimParaObj);
  //   if(SimParaObj.getTransPathFeasiFlag()){
  //     EndEffectorPathInfo EndEffectorPathObj(CubicSplineInfoObj);
  //     ControlReferenceObj = TrajectoryPlanning( SimRobotInner, InvertedPendulumObj, RMObject, SelfLinkGeoObj,
  //                                               EndEffectorPathObj, OneHandAlreadyFlag, SimParaObj);
  //       if(ControlReferenceObj.getReadyFlag()) break;
  //     }
  //   }
    return ControlReferenceObj;
  }