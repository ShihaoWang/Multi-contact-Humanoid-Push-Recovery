#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

static Robot SimRobotObj;
static int SwingLinkInfoIndex;
static std::vector<int> SwingLinkChain;
static Vector3            GoalPos;
static QuaternionRotation GoalQuat;
static std::vector<double> ReferenceConfig;
static SelfLinkGeoInfo SelfLinkGeoObj;
static double Ko = 10.0;

struct IKQuaternionOpt: public NonlinearOptimizerInfo
{
  IKQuaternionOpt():NonlinearOptimizerInfo(){};

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

      std::vector<double> F_val = IKQuaternionOptNCons(*n, *neF, x_vec);
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
  static std::vector<double> IKQuaternionOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & SwingLinkConfig)
  {
    // This funciton provides the constraint for the configuration variable
    std::vector<double> F(nObjNCons);
    for (int i = 0; i < SwingLinkChain.size(); i++)
        ReferenceConfig[SwingLinkChain[i]] = SwingLinkConfig[i];
    SimRobotObj.UpdateConfig(Config(ReferenceConfig));
    SimRobotObj.UpdateGeometry();

    /* Position Feedback */
    Vector3 EndEffectorAvgPos;
    SimRobotObj.GetWorldPosition( NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].AvgLocalContact,
                                  NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex,
                                  EndEffectorAvgPos);
    Vector3 PosOffset = EndEffectorAvgPos - GoalPos;
    
    /* Orientation Feedback */
    QuaternionRotation CurrentQuat = getEndEffectorQuaternion(SimRobotObj, SwingLinkInfoIndex);

    double eta1 = GoalQuat.data[0];
    double q1_x = GoalQuat.data[1];
    double q1_y = GoalQuat.data[2];
    double q1_z = GoalQuat.data[3];

    double eta2 = CurrentQuat.data[0];
    double q2_x = CurrentQuat.data[1];
    double q2_y = CurrentQuat.data[2];
    double q2_z = CurrentQuat.data[3];

    Vector3 q1(q1_x, q1_y, q1_z);
    Vector3 q2(q2_x, q2_y, q2_z);

    Vector3 delta_q = eta1 * q2 - eta2 * q1 - cross(q1, q2);
    Vector3 OriOffset = delta_q;

    F[0] = PosOffset.normSquared() + Ko * OriOffset.normSquared();

    int ConstraintIndex = 1;
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
    return F;
  }
};
std::vector<double> IKQuaternion(const Robot & SimRobot, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj_, const ControlReferenceInfo  & ControlReference, double InnerTime, bool & IKFlag){
  // This function is used to calculate robot's current reference configuration given its end effector trajectory.
  SimRobotObj = SimRobot;
  SwingLinkInfoIndex = ControlReference.getSwingLinkInfoIndex();
  SwingLinkChain = SwingLinkChainSelector(RMObject, SwingLinkInfoIndex, ControlReference.getOneHandAlreadyFlag());
  Vector EndEffectorGlobalPosRefVec;
  ControlReference.EndEffectorTraj.Eval(InnerTime, EndEffectorGlobalPosRefVec);
  Vector3 EndEffectorGlobalPosRef(EndEffectorGlobalPosRefVec);
  QuaternionRotation EndEffectorQuatRef = QuaternionRotationReference(InnerTime, ControlReference);
  
  GoalPos = EndEffectorGlobalPosRef;
  GoalQuat = EndEffectorQuatRef;
  ReferenceConfig = SimRobot.q;
  SelfLinkGeoObj = SelfLinkGeoObj_;

  IKQuaternionOpt IKQuaternionOptProblem;

  // Static Variable Substitution
  std::vector<double> SwingLinkChainGuess(SwingLinkChain.size());
  int n = SwingLinkChain.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF += SwingLinkChain.size()-3;                                               // Self-Collision Avoidance
  IKQuaternionOptProblem.InnerVariableInitialize(n, neF);

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
  IKQuaternionOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < neF; i++){
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }
  IKQuaternionOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  IKQuaternionOptProblem.SeedGuessUpdate(SwingLinkChainGuess);

  /*
    Given a name of this problem for the output
  */
  IKQuaternionOptProblem.ProblemNameUpdate("IKQuaternionOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  IKQuaternionOptProblem.NonlinearProb.setIntParameter("Iterations limit", 500);
  IKQuaternionOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 50);
  IKQuaternionOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  IKQuaternionOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  IKQuaternionOptProblem.ProblemOptionsUpdate(0, 3);
  IKQuaternionOptProblem.Solve(SwingLinkChainGuess);

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
  IKFlag = true;
  if(SelfCollisionDistTol<-0.0025){
      std::printf("IKQuaternionOptimazation Failure due to Self-collision for Link %d! \n", NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex);
      IKFlag = false;
  }

  // std::string ConfigPath = "./";
  // std::string OptConfigFile = "IKOptConfig.config";
  // RobotConfigWriter(OptConfig, ConfigPath, OptConfigFile);
  return OptConfig;

}