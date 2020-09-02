#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

static Robot SimRobotObj;
static int SwingLinkInfoIndex;
static Vector3 GoalDir;
static std::vector<double> ReferenceConfig;
static double EndEffectorProjx;
static double EndEffectorProjy;
static int EndEffectorIndexA, EndEffectorIndexB;

struct EndEffectorOriOpt: public NonlinearOptimizerInfo
{
  EndEffectorOriOpt():NonlinearOptimizerInfo(){};

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

      std::vector<double> F_val = EndEffectorOriOptNCons(*n, *neF, x_vec);
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
  static std::vector<double> EndEffectorOriOptNCons(const int & nVar, const int & nObjNCons, const std::vector<double> & SwingLinkConfig)
  {
    // This funciton provides the constraint for the configuration variable
    std::vector<double> F(nObjNCons);
    ReferenceConfig[EndEffectorIndexA] = SwingLinkConfig[0];
    ReferenceConfig[EndEffectorIndexB] = SwingLinkConfig[1];
    SimRobotObj.UpdateConfig(Config(ReferenceConfig));

    RobotLink3D EndEffectorLink = SimRobotObj.links[NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex];
    Vector3 EndEffectorInitxDir, EndEffectorInityDir;   // Eventually these two directions should be orthgonal to goal direction.
    EndEffectorInitxDir.x = EndEffectorLink.T_World.R.data[0][0];
    EndEffectorInitxDir.y = EndEffectorLink.T_World.R.data[0][1];
    EndEffectorInitxDir.z = EndEffectorLink.T_World.R.data[0][2];
    double Proj_x = EndEffectorInitxDir.dot(GoalDir);

    EndEffectorInityDir.x = EndEffectorLink.T_World.R.data[1][0];
    EndEffectorInityDir.y = EndEffectorLink.T_World.R.data[1][1];
    EndEffectorInityDir.z = EndEffectorLink.T_World.R.data[1][2];
    double Proj_y = EndEffectorInityDir.dot(GoalDir);

    F[0]  = (Proj_x - EndEffectorProjx) * (Proj_x - EndEffectorProjx);
    F[0] += (Proj_y - EndEffectorProjy) * (Proj_y - EndEffectorProjy);

    F[1] = 0.0;
    return F;
  }
};

std::vector<double> EndEffectorOriOptimazation(const Robot & SimRobot, int SwingLinkInfoIndex_, int EndEffectorIndexA_, int EndEffectorIndexB_, double EndEffectorProjx_, double EndEffectorProjy_, Vector3 GoalDir_){
  // This function is used to calculate robot's current reference configuration given its end effector trajectory.
  SimRobotObj = SimRobot;

  SwingLinkInfoIndex = SwingLinkInfoIndex_;
  EndEffectorIndexA = EndEffectorIndexA_;
  EndEffectorIndexB = EndEffectorIndexB_;

  EndEffectorProjx  = EndEffectorProjx_;
  EndEffectorProjy  = EndEffectorProjy_;

  GoalDir = GoalDir_;
  ReferenceConfig = SimRobot.q;

  EndEffectorOriOpt EndEffectorOriOptProblem;

  // Static Variable Substitution
  std::vector<double> SwingLinkChainGuess(2);
  SwingLinkChainGuess[0] = ReferenceConfig[EndEffectorIndexA];
  SwingLinkChainGuess[1] = ReferenceConfig[EndEffectorIndexB];

  int n = 2;

  // Cost function on the norm difference between the reference avg position and the modified contact position.
  int neF = 1;
  neF += 1;                                                                     // Redundant
  EndEffectorOriOptProblem.InnerVariableInitialize(n, neF);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> xlow_vec(n), xupp_vec(n);
  xlow_vec[0] = SimRobot.qMin(EndEffectorIndexA_);
  xupp_vec[0] = SimRobot.qMax(EndEffectorIndexA_);
  
  xlow_vec[1] = SimRobot.qMin(EndEffectorIndexB_);
  xupp_vec[1] = SimRobot.qMax(EndEffectorIndexB_);

  EndEffectorOriOptProblem.VariableBoundsUpdate(xlow_vec, xupp_vec);

  /*
    Initialize the bounds of variables
  */
  std::vector<double> Flow_vec(neF), Fupp_vec(neF);
  for (int i = 0; i < neF; i++){
    Flow_vec[i] = 0;
    Fupp_vec[i] = 1e10;
  }

  EndEffectorOriOptProblem.ConstraintBoundsUpdate(Flow_vec, Fupp_vec);

  /*
    Initialize the seed guess
  */
  EndEffectorOriOptProblem.SeedGuessUpdate(SwingLinkChainGuess);

  /*
    Given a name of this problem for the output
  */
  EndEffectorOriOptProblem.ProblemNameUpdate("EndEffectorOriOptProblem", 0);

  // Here we would like allow much more time to be spent on IK
  EndEffectorOriOptProblem.NonlinearProb.setIntParameter("Iterations limit", 500);
  EndEffectorOriOptProblem.NonlinearProb.setIntParameter("Major iterations limit", 50);
  EndEffectorOriOptProblem.NonlinearProb.setIntParameter("Major print level", 0);
  EndEffectorOriOptProblem.NonlinearProb.setIntParameter("Minor print level", 0);
  /*
    ProblemOptions seting
  */
  // Solve with Finite-Difference
  EndEffectorOriOptProblem.ProblemOptionsUpdate(0, 3);
  EndEffectorOriOptProblem.Solve(SwingLinkChainGuess);

  std::vector<double> OptConfig = ReferenceConfig;

  OptConfig[EndEffectorIndexA] = SwingLinkChainGuess[0];
  OptConfig[EndEffectorIndexB] = SwingLinkChainGuess[1];

  SimRobotObj.UpdateConfig(Config(OptConfig));
  SimRobotObj.UpdateGeometry();

  return OptConfig;
}
