#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

static Robot SimRobotObj;
static int SwingLinkInfoIndex;
static std::vector<int> SwingLinkChain;
static Vector3 GoalPos;
static Vector3 GoalDir;
static std::vector<double> ReferenceConfig;
static SelfLinkGeoInfo SelfLinkGeoObj;
static double EndEffectorTol = 1e-3;

struct IKOrientationConfigOpt: public NonlinearOptimizerInfo
{
  IKOrientationConfigOpt():NonlinearOptimizerInfo(){};

  // This struct inherits the NonlinearOptimizerInfo struct and we just need to defined the Constraint function
  static void ObjNConstraint(int    *Status, int *n,    double x[],
    int    *needF,  int *neF,  double F[],
    int    *needG,  int *neG,  double G[],
    char      *cu,  int *lencu,
    int    iu[],    int *leniu,
    double ru[],    int *lenru)
    {
      std::vector<double> x_vec(*n);
      for (int i = 0; i < *n; i++)
        x_vec[i] = x[i];

      std::vector<double> F_val = IKOrientationConfigOptNCons(*n, *neF, x_vec);
      for (int i = 0; i < *neF; i++)
        F[i] = F_val[i];
    }
  void Solve(std::vector<double> &RobotConfig)
  {
    int StartType = 0;
      NonlinearProb.solve(StartType, neF, n, ObjAdd, ObjRow, ObjNConstraint,
      xlow, xupp, Flow, Fupp,
      x, xstate, xmul, F, Fstate, Fmul,
      nS, nInf, sumInf);
      for (int i = 0; i < n; i++)
        RobotConfig[i] = x[i];

      delete []x;      delete []xlow;   delete []xupp;
      delete []xmul;   delete []xstate;

      delete []F;      delete []Flow;   delete []Fupp;
      delete []Fmul;   delete []Fstate;
  }
  static std::vector<double> IKOrientationConfigOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & SwingLinkConfig)
  {
    // This funciton provides the constraint for the configuration variable
    std::vector<double> F(nObjNCons);
    for (int i = 0; i < SwingLinkChain.size(); i++)
        ReferenceConfig[SwingLinkChain[i]] = SwingLinkConfig[i];
    SimRobotObj.UpdateConfig(Config(ReferenceConfig));
    SimRobotObj.UpdateGeometry();
    Vector3 EndEffectorAvgPos;
    SimRobotObj.GetWorldPosition( NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].AvgLocalContact,
                                  NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex,
                                  EndEffectorAvgPos);
    Vector3 AvgDiff = EndEffectorAvgPos - GoalPos;
    F[0] = AvgDiff.normSquared();
    int ConstraintIndex = 1;
    for (int i = 0; i < NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LocalContacts.size(); i++)
    {
      Vector3 LinkiPjPos;
      SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LocalContacts[i], NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex, LinkiPjPos);
      F[ConstraintIndex] = SDFInfo.SignedDistance(LinkiPjPos);
      ConstraintIndex+=1;
    }

    // Self-collision constraint
    std::vector<double> SelfCollisionDistVec(SwingLinkChain.size()-3);
    for (int i = 0; i < SwingLinkChain.size() - 3; i++){
      Box3D Box3DObj = SimRobotObj.geometry[SwingLinkChain[i]]->GetBB();
      std::vector<Vector3> BoxVerticesVec = BoxVertices(Box3DObj);
      std::vector<double> DistVec(BoxVerticesVec.size());
      for (int j = 0; j < BoxVerticesVec.size(); j++)
        DistVec[j] = SelfLinkGeoObj.SelfCollisionDist(SwingLinkInfoIndex, BoxVerticesVec[j]);
      SelfCollisionDistVec[i] = *std::min_element(DistVec.begin(), DistVec.end());
      F[ConstraintIndex] = SelfCollisionDistVec[i];
      ConstraintIndex+=1;
    }
    RobotLink3D EndEffectorLink = SimRobotObj.links[NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex];

    Vector3 EndEffectorInitxDir, EndEffectorInityDir;   // Eventually these two directions should be orthgonal to goal direction.
    EndEffectorInitxDir.x = EndEffectorLink.T_World.R.data[0][0];
    EndEffectorInitxDir.y = EndEffectorLink.T_World.R.data[0][1];
    EndEffectorInitxDir.z = EndEffectorLink.T_World.R.data[0][2];
    F[ConstraintIndex] = EndEffectorInitxDir.dot(GoalDir);
    ConstraintIndex+=1;

    EndEffectorInityDir.x = EndEffectorLink.T_World.R.data[1][0];
    EndEffectorInityDir.y = EndEffectorLink.T_World.R.data[1][1];
    EndEffectorInityDir.z = EndEffectorLink.T_World.R.data[1][2];
    F[ConstraintIndex] = EndEffectorInityDir.dot(GoalDir);
    ConstraintIndex+=1;

    return F;
  }
};

std::vector<double> IKOrientationConfigOptimazation(const Robot & SimRobot, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj_, Vector3 GoalPos_, Vector3 GoalDir_, int SwingLinkInfoIndex_, double sCur, double EndEffectorProjx, double EndEffectorProjy, bool & OptFlag){
  // This function is used to calculate robot's current reference configuration given its end effector trajectory.
  SimRobotObj = SimRobot;
  SwingLinkInfoIndex = SwingLinkInfoIndex_;
  SwingLinkChain = RMObject.EndEffectorLink2Pivotal[SwingLinkInfoIndex];
  GoalPos = GoalPos_;
  GoalDir = GoalDir_;
  ReferenceConfig = SimRobot.q;

  SelfLinkGeoObj = SelfLinkGeoObj_;
  IKOrientationConfigOpt IKOrientationConfigOptProblem;

  // Static Variable Substitution
  std::vector<double> SwingLinkChainGuess(SwingLinkChain.size());
  int n = SwingLinkChain.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF += NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LocalContacts.size();
  neF += SwingLinkChain.size()-3;                                               // Self-Collision Avoidance
  neF += 2;                                                                     // End Effector Orientation Constraint
  IKOrientationConfigOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);
  for (int i = 0; i < n; i++)
  {
    // Configuration
    xlow_vec[i] = SimRobot.qMin(SwingLinkChain[i]);
    xupp_vec[i] = SimRobot.qMax(SwingLinkChain[i]);
    SwingLinkChainGuess[i] = ReferenceConfig[SwingLinkChain[i]];
  }
  IKOrientationConfigOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < neF; i++)
  {
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }

  Flow_vec[neF-2] = EndEffectorProjx - EndEffectorTol;
  Fupp_vec[neF-2] = EndEffectorProjx + EndEffectorTol;

  Flow_vec[neF-1] = EndEffectorProjy - EndEffectorTol;
  Fupp_vec[neF-1] = EndEffectorProjy + EndEffectorTol;

  IKOrientationConfigOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  IKOrientationConfigOptProblem.SeedGuessUpdate(SwingLinkChainGuess);

  /*
    Given a name of this problem for the output
  */
  IKOrientationConfigOptProblem.ProblemNameUpdate("IKOrientationConfigOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  IKOrientationConfigOptProblem.NonlinearProb.setIntParameter("Iterations limit", 500);
  IKOrientationConfigOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 50);
  IKOrientationConfigOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  IKOrientationConfigOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  IKOrientationConfigOptProblem.ProblemOptionsUpdate(0, 3);
  IKOrientationConfigOptProblem.Solve(SwingLinkChainGuess);

  std::vector<double> OptConfig = SimRobot.q;
  for (int i = 0; i < n; i++)
    OptConfig[SwingLinkChain[i]] = SwingLinkChainGuess[i];
  SimRobotObj.UpdateConfig(Config(OptConfig));
  SimRobotObj.UpdateGeometry();

  // Self-collision constraint numerical checker
  std::vector<double> SelfCollisionDistVec(SwingLinkChain.size()-3);
  for (int i = 0; i < SwingLinkChain.size()-3; i++)     // Due to the bounding box size of torso link
  {
    Box3D Box3DObj = SimRobotObj.geometry[SwingLinkChain[i]]->GetBB();
    std::vector<Vector3> BoxVerticesVec = BoxVertices(Box3DObj);
    std::vector<double> DistVec(BoxVerticesVec.size());
    for (int j = 0; j < BoxVerticesVec.size(); j++)
      DistVec[j] = SelfLinkGeoObj_.SelfCollisionDist(SwingLinkInfoIndex, BoxVerticesVec[j]);
    SelfCollisionDistVec[i] = *std::min_element(DistVec.begin(), DistVec.end());
  }
  double SelfCollisionDistTol = *std::min_element(SelfCollisionDistVec.begin(), SelfCollisionDistVec.end());

  OptFlag = true;
  if(SelfCollisionDistTol<-0.0025){
      // std::printf("IKOrientationConfigOptimazation Failure due to Self-collision for Link %d! \n", NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex);
      OptFlag = false;
  }

  Vector3 EndEffectorAvgPos;
  SimRobotObj.GetWorldPosition(NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].AvgLocalContact, NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex, EndEffectorAvgPos);
  Vector3 AvgDiff = EndEffectorAvgPos - GoalPos;
  double DistTestTol = 0.0625;
  double DistTest = AvgDiff.normSquared();
  double sTol = 1e-3;
  if(sCur * sCur>=(1.0 - sTol)){
    if(DistTest>DistTestTol){
      // std::printf("IKOrientationConfigOptimazation Failure due to Goal Contact Non-reachability for Link %d! \n", NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex);
      OptFlag = false;
    }
  }

//   std::string ConfigPath = "./";
//   std::string OptConfigFile = "IKOptConfig.config";
//   RobotConfigWriter(OptConfig, ConfigPath, OptConfigFile);
  return OptConfig;
}
