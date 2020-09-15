#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

extern std::vector<LinkInfo>   LinkInfoObj;
extern SDFInfo                 SDFInfoObj;

static Robot SimRobotObj;
static int SwingLinkInfoIndex;
static std::vector<int> SwingLinkChain;
static std::vector<double> InnerConfig;
static std::vector<double> FixedConfig;

static SelfCollisionInfo SelfCollisionInfoObj;

static double TouchDownTol = 0.001;   // 1mm

struct TouchDownConfigOpt: public NonlinearOptimizerInfo{
  TouchDownConfigOpt():NonlinearOptimizerInfo(){};

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

      std::vector<double> F_val = TouchDownConfigOptNCons(*n, *neF, x_vec);
      for (int i = 0; i < *neF; i++)
        F[i] = F_val[i];
    }
  void Solve(std::vector<double> &RobotConfig){
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
  static std::vector<double> TouchDownConfigOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & SwingLinkConfig){
    std::vector<double> F(nObjNCons);
    double ConfigDiff = 0.0;
    for (int i = 0; i < SwingLinkChain.size(); i++){
      double ConfigDiff_i = FixedConfig[SwingLinkChain[i]] - SwingLinkConfig[i];
      ConfigDiff+=ConfigDiff_i * ConfigDiff_i;
      InnerConfig[SwingLinkChain[i]] = SwingLinkConfig[i];
    }
    F[0] = ConfigDiff;
    SimRobotObj.UpdateConfig(Config(InnerConfig));

    int ConstraintIndex = 1;
    // Self-collision constraint
    for (int i = 0; i < LinkInfoObj[SwingLinkInfoIndex].LocalContacts.size(); i++){
      Vector3 LinkiPjPos;
      SimRobotObj.GetWorldPosition(LinkInfoObj[SwingLinkInfoIndex].LocalContacts[i], LinkInfoObj[SwingLinkInfoIndex].LinkIndex, LinkiPjPos);
      F[ConstraintIndex] = SDFInfoObj.SignedDistance(LinkiPjPos);
      ConstraintIndex+=1;
    }
    return F;
  }
};

std::vector<double> TouchDownConfigOptimazation(const Robot & SimRobot, int SwingLinkInfoIndexInner, const std::vector<int> & SwingLinkChainInner, const SelfCollisionInfo & SelfCollisionInfoObjInner, const SimPara & SimParaObj, bool & TouchDownOptFlag){
  // This function is used to optimize robot's configuration such that a certain contact can be reached for that end effector.
  SimRobotObj         = SimRobot;
  SwingLinkInfoIndex  = SwingLinkInfoIndexInner;
  SwingLinkChain      = SwingLinkChainInner;

  InnerConfig = SimRobot.q;
  FixedConfig = SimRobot.q;
  SelfCollisionInfoObj = SelfCollisionInfoObjInner;

  TouchDownConfigOpt TouchDownConfigOptProblem;

  // Static Variable Substitution
  std::vector<double> SwingLinkChainGuess(SwingLinkChain.size());
  int n = SwingLinkChain.size();
  int SwingLinkContactSize = LinkInfoObj[SwingLinkInfoIndex].LocalContacts.size();

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF += SwingLinkContactSize;
  TouchDownConfigOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);
  for (int i = 0; i < n; i++){
    // Configuration
    xlow_vec[i] = SimRobot.qMin(SwingLinkChain[i]);
    xupp_vec[i] = SimRobot.qMax(SwingLinkChain[i]);
    SwingLinkChainGuess[i] = InnerConfig[SwingLinkChain[i]];
  }
  TouchDownConfigOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < neF; i++){
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }
  for (int i = neF - SwingLinkContactSize; i < neF; i++) {
    Flow_vec[i] = 0.0;
    Fupp_vec[i] = TouchDownTol;
  }
  TouchDownConfigOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  TouchDownConfigOptProblem.SeedGuessUpdate(SwingLinkChainGuess);

  /*
    Given a name of this problem for the output
  */
  TouchDownConfigOptProblem.ProblemNameUpdate("TouchDownConfigOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  TouchDownConfigOptProblem.NonlinearProb.setIntParameter("Iterations limit", 250);
  TouchDownConfigOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 50);
  TouchDownConfigOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  TouchDownConfigOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  TouchDownConfigOptProblem.ProblemOptionsUpdate(0, 3);
  TouchDownConfigOptProblem.Solve(SwingLinkChainGuess);

  std::vector<double> OptConfig = InnerConfig;

  for (int i = 0; i < n; i++)
    OptConfig[SwingLinkChain[i]] = SwingLinkChainGuess[i];
  
  SimRobotObj.UpdateConfig(Config(OptConfig));
  SimRobotObj.UpdateGeometry();

  TouchDownOptFlag = true;
  Vector3 EndEffectorAvgPos;
  SimRobotObj.GetWorldPosition( LinkInfoObj[SwingLinkInfoIndex].AvgLocalContact, 
                                LinkInfoObj[SwingLinkInfoIndex].LinkIndex, 
                                EndEffectorAvgPos);
  double EndEffectorDist = SDFInfoObj.SignedDistance(EndEffectorAvgPos);
  if(TouchDownTol * TouchDownTol<=EndEffectorDist * EndEffectorDist){
      std::printf("LastStageOptimization Failure due to Goal Contact Too High for Link %d! with Distance %f\n", 
                    LinkInfoObj[SwingLinkInfoIndex].LinkIndex, EndEffectorDist);
      TouchDownOptFlag = false;
  }
  return OptConfig;
}
