#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

extern std::vector<LinkInfo>   LinkInfoObj;
extern SDFInfo                 SDFInfoObj;

static Robot              SimRobotObj;
static int                SwingLinkInfoIndex;
static std::vector<int>   SwingLinkChain;
static SelfCollisionInfo  SelfCollisionInfoObj;

static Vector3            GoalPos;
static Vector3            GoalDir;

static std::vector<double> InnerConfig;
static std::vector<double> FixedConfig;

static double EndEffectorTol = 1e-3;
static double ConfigCoeff = 0.0;

struct StageIKOpt: public NonlinearOptimizerInfo
{
  StageIKOpt():NonlinearOptimizerInfo(){};

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

      std::vector<double> F_val = StageIKOptNCons(*n, *neF, x_vec);
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
  static std::vector<double> StageIKOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & SwingLinkConfig)
  {
    // This funciton provides the constraint for the configuration variable
    double ConfigDiff = 0.0;
    std::vector<double> F(nObjNCons);
    for (int i = 0; i < SwingLinkChain.size(); i++){
      InnerConfig[SwingLinkChain[i]] = SwingLinkConfig[i];
      double ConfigDiff_i = InnerConfig[SwingLinkChain[i]] - FixedConfig[SwingLinkChain[i]];
      ConfigDiff+= ConfigCoeff * ConfigDiff_i * ConfigDiff_i;
    }
    SimRobotObj.UpdateConfig(Config(InnerConfig));
    SimRobotObj.UpdateGeometry();

    Vector3 EndEffectorAvgPos;
    SimRobotObj.GetWorldPosition( LinkInfoObj[SwingLinkInfoIndex].AvgLocalContact,
                                  LinkInfoObj[SwingLinkInfoIndex].LinkIndex,
                                  EndEffectorAvgPos);
    Vector3 AvgDiff = EndEffectorAvgPos - GoalPos;
    F[0] = AvgDiff.normSquared() + ConfigDiff;

    int ConstraintIndex = 1;
    // Environment Penetration Constraint
    for (int i = 0; i < LinkInfoObj[SwingLinkInfoIndex].LocalContacts.size(); i++)
    {
      Vector3 LinkiPjPos;
      SimRobotObj.GetWorldPosition( LinkInfoObj[SwingLinkInfoIndex].LocalContacts[i], 
                                    LinkInfoObj[SwingLinkInfoIndex].LinkIndex, 
                                    LinkiPjPos);
      F[ConstraintIndex] = SDFInfoObj.SignedDistance(LinkiPjPos);
      ConstraintIndex+=1;
    }

    // Self-collision constraint
    std::vector<double> SelfCollisionDistVec(SwingLinkChain.size()-3);
    for (int i = 0; i < SwingLinkChain.size()-3; i++){
      Box3D Box3DObj = SimRobotObj.geometry[SwingLinkChain[i]]->GetBB();
      std::vector<Vector3> BoxVerticesVec = BoxVertices(Box3DObj);
      std::vector<double> DistVec(BoxVerticesVec.size());
      for (int j = 0; j < BoxVerticesVec.size(); j++)
        DistVec[j] = SelfCollisionInfoObj.getSelfCollisionDist(SwingLinkInfoIndex, BoxVerticesVec[j]);
      SelfCollisionDistVec[i] = *std::min_element(DistVec.begin(), DistVec.end());
      F[ConstraintIndex] = SelfCollisionDistVec[i];
      ConstraintIndex+=1;
    }

    RobotLink3D EndEffectorLink = SimRobotObj.links[LinkInfoObj[SwingLinkInfoIndex].LinkIndex];

    Vector3 EndEffectorxDir, EndEffectoryDir, EndEffectorzDir;   // Eventually these two directions should be orthgonal to goal direction.
    EndEffectorxDir.x = EndEffectorLink.T_World.R.data[0][0];
    EndEffectorxDir.y = EndEffectorLink.T_World.R.data[0][1];
    EndEffectorxDir.z = EndEffectorLink.T_World.R.data[0][2];
    F[ConstraintIndex] = EndEffectorxDir.dot(GoalDir);
    ConstraintIndex+=1;

    EndEffectoryDir.x = EndEffectorLink.T_World.R.data[1][0];
    EndEffectoryDir.y = EndEffectorLink.T_World.R.data[1][1];
    EndEffectoryDir.z = EndEffectorLink.T_World.R.data[1][2];
    F[ConstraintIndex] = EndEffectoryDir.dot(GoalDir);
    ConstraintIndex+=1;

    EndEffectorzDir.x = EndEffectorLink.T_World.R.data[2][0];
    EndEffectorzDir.y = EndEffectorLink.T_World.R.data[2][1];
    EndEffectorzDir.z = EndEffectorLink.T_World.R.data[2][2];
    F[ConstraintIndex] = EndEffectoryDir.dot(GoalDir);
    ConstraintIndex+=1;

    return F;
  }
};

std::vector<double> StageIKOptimization(const Robot & SimRobotInner, const int & SwingLinkInfoIndexInner,
                                        const std::vector<int> & SwingLinkChainInner, 
                                        const SelfCollisionInfo & SelfCollisionInfoObjInner,  
                                        const std::vector<Vector3> & StageIKVector3Vec, 
                                        const std::vector<double> & StageIKVec, bool & StageIKFlag){
  // This function is used to optimize robot's configuration such that a certain contact can be reached for that end effector.
  SimRobotObj           = SimRobotInner;
  SwingLinkInfoIndex    = SwingLinkInfoIndexInner;
  SwingLinkChain        = SwingLinkChainInner;
  SelfCollisionInfoObj  = SelfCollisionInfoObjInner;

  GoalPos = StageIKVector3Vec[0];
  GoalDir = StageIKVector3Vec[1];

  double sVal      = StageIKVec[0];
  double EndEffectorProjx = StageIKVec[1];
  double EndEffectorProjy = StageIKVec[2];
  double EndEffectorProjz = StageIKVec[3]; 

  InnerConfig = SimRobotInner.q;             // Used for Inner Optimization
  FixedConfig = SimRobotInner.q;             // Used for comparison with initial fixed configuration

  StageIKOpt StageIKOptProblem;

  // Static Variable Substitution
  std::vector<double> SwingLinkChainGuess(SwingLinkChain.size());
  int n = SwingLinkChain.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF += LinkInfoObj[SwingLinkInfoIndex].LocalContacts.size();
  neF += SwingLinkChain.size()-3;                                               // Self-Collision Avoidance
  neF += 3;                                                                     // End Effector Orientation Constraint

  StageIKOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);
  for (int i = 0; i < n; i++){
    // Configuration
    xlow_vec[i] = SimRobotInner.qMin(SwingLinkChain[i]);
    xupp_vec[i] = SimRobotInner.qMax(SwingLinkChain[i]);
    SwingLinkChainGuess[i] = FixedConfig[SwingLinkChain[i]];
  }
  StageIKOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < neF; i++){
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }

  // When SwingLinkInfoIndex is 2 or 3, hand gets involved and then the constraint is on robot's z
  if(SwingLinkInfoIndex<2){
    // Foot Contact Configuration
    Flow_vec[neF-3] = EndEffectorProjx - EndEffectorTol;
    Fupp_vec[neF-3] = EndEffectorProjx + EndEffectorTol;

    Flow_vec[neF-2] = EndEffectorProjy - EndEffectorTol;
    Fupp_vec[neF-2] = EndEffectorProjy + EndEffectorTol;

    Flow_vec[neF-1] = -1e10;
    Fupp_vec[neF-1] = 1e10;

  } else {
    // Hand Contact Configuration
    Flow_vec[neF-3] = -1e10;
    Fupp_vec[neF-3] = 1e10;

    Flow_vec[neF-2] = -1e10;
    Fupp_vec[neF-2] = 1e10;

    Flow_vec[neF-1] = EndEffectorProjz - EndEffectorTol;
    Fupp_vec[neF-1] = EndEffectorProjz + EndEffectorTol;
  }
  StageIKOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  StageIKOptProblem.SeedGuessUpdate(SwingLinkChainGuess);

  /*
    Given a name of this problem for the output
  */
  StageIKOptProblem.ProblemNameUpdate("StageIKOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  StageIKOptProblem.NonlinearProb.setIntParameter("Iterations limit", 250);
  StageIKOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 25);
  StageIKOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  StageIKOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  StageIKOptProblem.ProblemOptionsUpdate(0, 3);
  StageIKOptProblem.Solve(SwingLinkChainGuess);

  std::vector<double> OptConfig = FixedConfig;
  for (int i = 0; i < n; i++)
    OptConfig[SwingLinkChain[i]] = SwingLinkChainGuess[i];
  
  SimRobotObj.UpdateConfig(Config(OptConfig));
  SimRobotObj.UpdateGeometry();

  StageIKFlag = StageIKOptimizationChecker(SimRobotObj, SwingLinkInfoIndex, SwingLinkChain, SelfCollisionInfoObj, GoalPos, OptConfig);

  return OptConfig;
}
