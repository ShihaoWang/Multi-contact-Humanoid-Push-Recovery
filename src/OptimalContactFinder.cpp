#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <queue>

extern std::vector<LinkInfo>   LinkInfoObj;
extern SDFInfo                 SDFInfoObj;
extern ReachabilityMap         ReachabilityMapObj;
extern AnyCollisionGeometry3D  TerrColGeomObj;

typedef std::pair<double, Vector3> qEle;

static std::vector<std::pair<Vector3, double>> ContactFreeInfoFn(const Robot & SimRobot, const std::vector<ContactStatusInfo> & RobotContactInfo){
  std::vector<std::pair<Vector3, double>> ContactFreeInfo;
  for (int i = 0; i < RobotContactInfo.size(); i++){
    if(RobotContactInfo[i].LocalContactStatus[0]){
      Vector3 LinkiPjPos;
      SimRobot.GetWorldPosition(LinkInfoObj[i].AvgLocalContact, LinkInfoObj[i].LinkIndex, LinkiPjPos);
      double Radius = ReachabilityMapObj.EndEffectorGeometryRadius[i];
      auto ContactFreeInfo_i = std::make_pair (LinkiPjPos, Radius);
      ContactFreeInfo.push_back(ContactFreeInfo_i);
    }
  }
  return ContactFreeInfo;
}

static std::vector<Vector3> SupportContactFinder(const PIPInfo & PIPObj, const std::vector<Vector3> & ContactFreeContact){
  Vector3 EdgeA = PIPObj.edge_a;
  Vector3 EdgeB = PIPObj.edge_b;
  Vector3 EdgeDir = EdgeB - EdgeA;
  std::vector<Vector3> SupportContact;
  SupportContact.reserve(ContactFreeContact.size());
  for (int i = 0; i < ContactFreeContact.size(); i++){
    Vector3 ContactFreePoint = ContactFreeContact[i];
    Vector3 PointNormDir = SDFInfoObj.SignedDistanceNormal(ContactFreePoint);
    Vector3 rPos2COMPos = ContactFreePoint - EdgeA;
    Vector3 InducedMomentum = cross(rPos2COMPos, PointNormDir);
    double  ProjMomentumVal = InducedMomentum.dot(EdgeDir);
    if(ProjMomentumVal<0.0) SupportContact.push_back(ContactFreePoint);
  }
  return SupportContact;
}

static std::vector<Vector3> OptimalContactFinder(const std::vector<Vector3> & SupportContact, const std::vector<Vector3> & FixedContacts, const Vector3 & COMPos, const Vector3 & COMVel, int CutOffNo, SimPara & SimParaObj){
  // This function selects the optimal contact given support contact.
  std::priority_queue<qEle, std::vector<qEle>, less<qEle> > OptimalContactQueue;
  std::vector<Vector3> CandidateContacts;
  std::vector<Vector3> CandidateContactWeights;
  std::vector<double> ContactFailureMetric(SupportContact.size());
  Vector3 CurContact = SimParaObj.getInitContactPos();
  LinkInfo SwingLinkInfo = LinkInfoObj[SimParaObj.getSwingLinkInfoIndex()];
  int RobotLinkIndex = SwingLinkInfo.LinkIndex;
  double ContactSelectionCoeff = SimParaObj.getContactSelectionCoeff();
  for (int i = 0; i < SupportContact.size(); i++){
    std::vector<Vector3> ActContacts = FixedContacts;
    ActContacts.push_back(SupportContact[i]);
    switch (RobotLinkIndex){      // Attach four vertices
    case 11:{
      for (int j = 0; j < SwingLinkInfo.LocalContacts.size(); j++){
        Vector3 Vertex = SupportContact[i] + SwingLinkInfo.LocalContacts[j] - SwingLinkInfo.AvgLocalContact;
        ActContacts.push_back(Vertex);
      }
    }
    break;
    case 17:{
      for (int j = 0; j < SwingLinkInfo.LocalContacts.size(); j++){
        Vector3 Vertex = SupportContact[i] + SwingLinkInfo.LocalContacts[j] - SwingLinkInfo.AvgLocalContact;
        ActContacts.push_back(Vertex);
      }
    }
    break;
    default:
      break;
    }
    std::vector<PIPInfo> PIPTotal = PIPGenerator(COMPos, COMVel, ActContacts);
    ContactFailureMetric[i] = FailureMetricEval(PIPTotal);
    if(ContactFailureMetric[i]>0.0){
      Vector3 ContactDiff = CurContact - SupportContact[i];
      double ContactDiffDist = ContactDiff.normSquared();
      double ContactDistCost = 1.0 * exp(-ContactSelectionCoeff * ContactDiffDist);
      ContactFailureMetric[i]*= ContactDistCost;
      OptimalContactQueue.push(std::make_pair(ContactFailureMetric[i], SupportContact[i]));
      CandidateContacts.push_back(SupportContact[i]);
      CandidateContactWeights.push_back(ContactFailureMetric[i] * SDFInfoObj.SignedDistanceNormal(SupportContact[i]));
    }
  }

  std::vector<Vector3> SelectedContacts;
  if(!CandidateContacts.size()){
    int OptiIndex = std::distance(ContactFailureMetric.begin(), std::max_element(ContactFailureMetric.begin(), ContactFailureMetric.end()));
    CandidateContacts.push_back(SupportContact[OptiIndex]);
    CandidateContactWeights.push_back(0.0);
    SelectedContacts.push_back(SupportContact[OptiIndex]);
  } else {
    int OptimalContactNumber = CandidateContacts.size();
    int OptEleNo = std::min(OptimalContactNumber, CutOffNo);
    for (int i = 0; i < OptEleNo; i++) {
      SelectedContacts.push_back(OptimalContactQueue.top().second);
      OptimalContactQueue.pop();
    }
  }
  // Vector3Writer(CandidateContacts, "OptimalContact");
  // Vector3Writer(CandidateContactWeights, "OptimalContactWeights");
  SimParaObj.DataRecorderObj.setCCSData(CandidateContacts, CandidateContactWeights, SelectedContacts);
  return SelectedContacts;
}

std::vector<Vector3> OptimalContactSearcher(const Robot & SimRobot, const PIPInfo & PIPObj, const ContactForm & ContactFormObj, SimPara & SimParaObj, double ForwardTime){
  std::vector<Vector3> OptimalContact;
  Vector3 COMPos, COMVel;
  getCentroidalState(SimRobot, COMPos, COMVel);
  
  // InvertedPendulumInfo InvertedPendulumObj(PIPObj.L, PIPObj.g, PIPObj.theta, PIPObj.thetadot, COMPos, COMVel);
  // InvertedPendulumObj.setEdges(PIPObj.edge_a, PIPObj.edge_b);
  
  // double ForwardTime = SimParaObj.ForwardDurationSeed;
     
  // bool MotionFlag = true;
  // Config UpdatedConfig  = WholeBodyDynamicsIntegrator(SimRobot, InvertedPendulumObj, ForwardTime, MotionFlag);
  // SimRobot.UpdateConfig(UpdatedConfig);
  // if(!MotionFlag){
  //   return OptimalContact;
  // } 

  // COMPos = InvertedPendulumObj.COMPos;
  // COMVel = InvertedPendulumObj.COMVel;

  //  0. Reachable with respect to the pivotal joint
  std::vector<Vector3> ReachableContacts = ReachabilityMapObj.ReachablePointsFinder(SimRobot, ContactFormObj.SwingLinkInfoIndex, SDFInfoObj);
  if(!ReachableContacts.size()) return OptimalContact;

  // 1. Self-collision from other end effectors
  std::vector<std::pair<Vector3, double>> ContactFreeInfoVec = ContactFreeInfoFn(SimRobot, ContactFormObj.FixedContactStatusInfo);
  std::vector<Vector3> CollisionFreeContacts = ReachabilityMapObj.ContactFreePointsFinder(ReachabilityMapObj.EndEffectorGeometryRadius[ContactFormObj.SwingLinkInfoIndex], ReachableContacts, ContactFreeInfoVec);
  if(!CollisionFreeContacts.size()) return OptimalContact;

  // 2. Supportive
  std::vector<Vector3> SupportiveContacts = SupportContactFinder(PIPObj, CollisionFreeContacts);
  if(!SupportiveContacts.size()) return OptimalContact;

  SimParaObj.DataRecorderObj.setRCSData(ReachableContacts, CollisionFreeContacts, SupportiveContacts);

  // Vector3Writer(ReachableContacts, "ReachableContacts");
  // Vector3Writer(CollisionFreeContacts, "CollisionFreeContacts");
  // Vector3Writer(SupportiveContacts, "SupportiveContacts");

  std::vector<Vector3> FixedContactPos;
  for (int i = 0; i < LinkInfoObj.size(); i++){
    for (int j = 0; j < LinkInfoObj[i].LocalContacts.size(); j++){
      if(ContactFormObj.FixedContactStatusInfo[i].LocalContactStatus[j]){
        Vector3 LinkiPjPos;
        SimRobot.GetWorldPosition(LinkInfoObj[i].LocalContacts[j], LinkInfoObj[i].LinkIndex, LinkiPjPos);
        FixedContactPos.push_back(LinkiPjPos);
      }
    }
  }

  // 3. Optimal Contact
  int CutOffNo = 5;
  OptimalContact = OptimalContactFinder(SupportiveContacts, FixedContactPos, COMPos, COMVel, CutOffNo, SimParaObj);
  if(!OptimalContact.size()) return OptimalContact;

  return OptimalContact;
}