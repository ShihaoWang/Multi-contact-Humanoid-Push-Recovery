#include "CommonHeader.h"

static std::vector<double> CubicSpline1DCoeff(double InitPos, double InitSlope, double GoalPos, double GoalSlope){
  // Cubic Spline Along 1-D: y(s) = a * s ^ 3 + b * s ^ 2 + c * s + d
  double sInit = 0.0;
  double sGoal = 1.0;

  // This function is used to generate Cubic Spline.
  double M00 = 3.0 * sInit * sInit;
  double M01 = 2.0 * sInit;
  double M02 = 1.0;

  double M10 = 3.0 * sGoal * sGoal;
  double M11 = 2.0 * sGoal;
  double M12 = 1.0;

  double M20 = (sInit * sInit * sInit - sGoal * sGoal * sGoal);
  double M21 = sInit * sInit - sGoal * sGoal;
  double M22 = sInit - sGoal;

  Vector3 Col1(M00, M10, M20);
  Vector3 Col2(M01, M11, M21);
  Vector3 Col3(M02, M12, M22);

  Matrix3 CubicMat(Col1, Col2, Col3);
  Vector3 CubicVec(InitSlope, GoalSlope, InitPos - GoalPos);

  Matrix3 CubicMatInv;
  CubicMat.getInverse(CubicMatInv);

  Vector3 abc = CubicMatInv * CubicVec;

  double a = abc.x;
  double b = abc.y;
  double c = abc.z;
  double d = InitPos - a * sInit * sInit * sInit - b * sInit * sInit - c * sInit;

  std::vector<double> CubicSplineCoeff = { a, b, c, d };
  return CubicSplineCoeff;
}

static std::vector<Vector3> CubicSpline3DCoeff(const Vector3 & InitPos, const Vector3 & InitSlope, const Vector3 & GoalPos, const Vector3 & GoalSlope){

  std::vector<double> Coeff_x = CubicSpline1DCoeff(InitPos.x, InitSlope.x, GoalPos.x, GoalSlope.x);

  std::vector<double> Coeff_y = CubicSpline1DCoeff(InitPos.y, InitSlope.y, GoalPos.y, GoalSlope.y);

  std::vector<double> Coeff_z = CubicSpline1DCoeff(InitPos.z, InitSlope.z, GoalPos.z, GoalSlope.z);

  double ax = Coeff_x[0];  double bx = Coeff_x[1];  double cx = Coeff_x[2];  double dx = Coeff_x[3];
  double ay = Coeff_y[0];  double by = Coeff_y[1];  double cy = Coeff_y[2];  double dy = Coeff_y[3];
  double az = Coeff_z[0];  double bz = Coeff_z[1];  double cz = Coeff_z[2];  double dz = Coeff_z[3];

  Vector3 a(ax, ay, az);
  Vector3 b(bx, by, bz);
  Vector3 c(cx, cy, cz);
  Vector3 d(dx, dy, dz);

  std::vector<Vector3> Coeffs = {a, b, c, d};
  
  return Coeffs;
}

static std::vector<double> sVecComputation(double EdgeLength, int EdgePointSize, int MiddlePointSize){
  double EdgeUnit = EdgeLength/(1.0 * EdgePointSize - 1.0);
  double MiddleUnit = (1.0 - EdgeLength)/(1.0 * MiddlePointSize - 1.0);
  double s = 0.0;

  std::vector<double> InitPart  = LinearSpace(0.0, EdgeLength, EdgePointSize);
  std::vector<double> EndPart   = LinearSpace(1.0 - EdgeLength, 1.0, EdgePointSize);
  std::vector<double> MidPart   = LinearSpace(EdgeLength, 1.0 - EdgeLength, MiddlePointSize);
  int sSize = EdgePointSize * 2 + MiddlePointSize - 2;
  std::vector<double> sVec;
  sVec.reserve(sSize);
  for (int i = 0; i < InitPart.size()-1; i++){
    sVec.push_back(InitPart[i]);
  }
  
  for (int i = 0; i < MidPart.size()-1; i++){
    sVec.push_back(MidPart[i]);
  }

  for (int i = 0; i < EndPart.size(); i++){
    sVec.push_back(EndPart[i]);
  }  
  return sVec;
}

void InitialWayPointsComputation(const Robot & SimRobot, int SwingLinkInfoIndex, const SimPara & SimParaObj, std::vector<Vector3> & InitWayPoints, std::vector<double> & sVec){
  // This function generates an initial set of waypoints based on a cubic spline.
  Vector3 PathInitPos = SimParaObj.getInitContactPos();
  Vector3 PathGoalPos = SimParaObj.getGoalContactPos();
  Vector3 PathInitSlope, PathEndSlope;
  EndEffectorPathSlopeComputation(SimRobot, SwingLinkInfoIndex, PathInitSlope, PathEndSlope, SimParaObj);

  std::vector<Vector3> Coeffs = CubicSpline3DCoeff(PathInitPos, PathInitSlope, PathGoalPos, PathEndSlope);

  // Initial and End Waypoints should be dense compared to Waypoints in the middle.
  double EdgeLength       = 0.1;
  int    EdgePointSize    = 5;
  int    MiddlePointSize  = 5;
  
  sVec = sVecComputation(EdgeLength, EdgePointSize, MiddlePointSize);

  std::vector<Vector3> WayPoints(sVec.size());
  for (int i = 0; i < sVec.size(); i++){
    double s = sVec[i];
    WayPoints[i] = getPostionFromCubicCoeffs(s, Coeffs);
  }
  InitWayPoints = WayPoints;
  return;
}