#ifndef ROBOTINFO_H
#define ROBOTINFO_H
#include <KrisLibrary/robotics/RobotDynamics3D.h>
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <Interface/SimulationGUI.h>
#include <unsupported/Eigen/CXX11/Tensor>
#include <KrisLibrary/meshing/TriMeshTopology.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/geometry/PQP/src/PQP.h>
#include "Modeling/Paths.h"
#include <fstream>
#include "Spline.h"

struct LinkInfo {
  LinkInfo(){LinkIndex = -1;}
  LinkInfo(int LinkIndex_){ LinkIndex = LinkIndex_; }
  void AddLocalConact(Vector3 local_contact){ LocalContacts.push_back(local_contact); }
  void AvgContactUpdate(){
    switch (LocalContacts.size()){
      case 0: throw std::invalid_argument( "LocalContacts should have been initialized!" );
      break;
      default:{
        Vector3 SumLocalContacts(0.0, 0.0, 0.0);
        for (int i = 0; i < LocalContacts.size(); i++){
          SumLocalContacts.x = SumLocalContacts.x + LocalContacts[i].x;
          SumLocalContacts.y = SumLocalContacts.y + LocalContacts[i].y;
          SumLocalContacts.z = SumLocalContacts.z + LocalContacts[i].z;
        }
        AvgLocalContact.x = SumLocalContacts.x/LocalContacts.size();
        AvgLocalContact.y = SumLocalContacts.y/LocalContacts.size();
        AvgLocalContact.z = SumLocalContacts.z/LocalContacts.size();
      }
      break;
    }
  }
  int LinkIndex;
  std::vector<Vector3> LocalContacts;
  Vector3 AvgLocalContact;
};

struct SDFInfo{
  SDFInfo(){
    Envi_x_min = 0;           Envi_x_max = 0;
    Envi_y_min = 0;           Envi_y_max = 0;
    Envi_z_min = 0;           Envi_z_max = 0;
    Envi_x_unit = 0;          Envi_y_unit = 0;          Envi_z_unit = 0;
    Envi_x_length = 0;        Envi_y_length = 0;        Envi_z_length = 0;
    GridNo = 0;
  }
  SDFInfo(const Eigen::Tensor<double, 3>& _SDFTensor, const std::vector<double> &_SDFSpecs){
    SDFTensor = _SDFTensor;
    Envi_x_min = _SDFSpecs[0];          Envi_x_max = _SDFSpecs[1];
    Envi_y_min = _SDFSpecs[2];          Envi_y_max = _SDFSpecs[3];
    Envi_z_min = _SDFSpecs[4];          Envi_z_max = _SDFSpecs[5];
    Envi_x_unit = _SDFSpecs[6];         Envi_y_unit = _SDFSpecs[7];         Envi_z_unit = _SDFSpecs[8];
    Envi_x_length = _SDFSpecs[9];       Envi_y_length = _SDFSpecs[10];      Envi_z_length = _SDFSpecs[11];
    GridNo = (int)_SDFSpecs[12];
  }
  double SignedDistance(const Vector3 &Point) const{
    double x_FloatIndex = (Point.x - Envi_x_min)/Envi_x_unit * 1.0;
    double y_FloatIndex = (Point.y - Envi_y_min)/Envi_y_unit * 1.0;
    double z_FloatIndex = (Point.z - Envi_z_min)/Envi_z_unit * 1.0;

    int x_leftindex = std::floor(x_FloatIndex);
    int y_leftindex = std::floor(y_FloatIndex);
    int z_leftindex = std::floor(z_FloatIndex);

    if(x_leftindex<0){
      x_leftindex = 0;
    } else {
      if(x_leftindex>GridNo-2) {
        x_leftindex = GridNo-2;
      }
    }

    if(y_leftindex<0) {
      y_leftindex = 0;
    } else {
      if(y_leftindex>GridNo-2) {
        y_leftindex = GridNo-2;
      }
    }

    if(z_leftindex<0) {
      z_leftindex = 0;
    } else {
      if(z_leftindex>GridNo-2) {
        z_leftindex = GridNo-2;
      }
    }

    int x_rightindex = x_leftindex + 1;
    int y_rightindex = y_leftindex + 1;
    int z_rightindex = z_leftindex + 1;

    double valA = SDFTensor(x_leftindex, y_leftindex, z_leftindex);
    double valB = SDFTensor(x_rightindex, y_leftindex, z_leftindex);
    double valC = SDFTensor(x_rightindex, y_rightindex, z_leftindex);
    double valD = SDFTensor(x_leftindex, y_rightindex, z_leftindex);

    double valE = SDFTensor(x_leftindex, y_leftindex, z_rightindex);
    double valF = SDFTensor(x_rightindex, y_leftindex, z_rightindex);
    double valG = SDFTensor(x_rightindex, y_rightindex, z_rightindex);
    double valH = SDFTensor(x_leftindex, y_rightindex, z_rightindex);

    // Since this is a tri-linear interpolation, there are three ways to do the interpolation

    // Type1:
    // Along x-direction
    double valMAB = (x_FloatIndex - x_leftindex*1.0) * (valB - valA) + valA;
    double valMDC = (x_FloatIndex - x_leftindex*1.0) * (valC - valD) + valD;
    double valMEF = (x_FloatIndex - x_leftindex*1.0) * (valF - valE) + valE;
    double valMHG = (x_FloatIndex - x_leftindex*1.0) * (valG - valH) + valH;

    // Along y-drection
    double valMABDC = (y_FloatIndex - y_leftindex*1.0) * (valMDC - valMAB) + valMAB;
    double valMEFGH = (y_FloatIndex - y_leftindex*1.0) * (valMHG - valMEF) + valMEF;

    // Along z-direction
    double valMABCDEFHG = (z_FloatIndex - z_leftindex*1.0) * (valMEFGH - valMABDC) + valMABDC;

    // // Type2:
    // // Along y-drection
    // double valMAD = (y_FloatIndex - y_leftindex*1.0) * (valD - valA) + valA;
    // double valMBC = (y_FloatIndex - y_leftindex*1.0) * (valC - valB) + valB;
    // double valMEH = (y_FloatIndex - y_leftindex*1.0) * (valH - valE) + valE;
    // double valMFG = (y_FloatIndex - y_leftindex*1.0) * (valG - valF) + valF;
    //
    // //Along x-direction
    // double valMADBC = (x_FloatIndex - x_leftindex * 1.0) * (valMBC - valMAD) + valMAD;
    // double valMEHFG = (x_FloatIndex - x_leftindex * 1.0) * (valMFG - valMEH) + valMEH;
    //
    // //Along z-direction
    // double valMADBCEHFG = (z_FloatIndex - z_leftindex*1.0) * (valMEHFG - valMADBC) + valMADBC;

    return valMABCDEFHG;
  }
  Vector3 SignedDistanceNormal(const Vector3 &Point) const {
    // This function is used to calculate the (1 x 3) Jacobian matrix given the current Position
    // This function is used to compute the distance from a 3D point to the environment terrain
    // The first job is to figure out the nearest neighbours of the Points

    double x_FloatIndex = (Point.x - Envi_x_min)/Envi_x_unit * 1.0;
    double y_FloatIndex = (Point.y - Envi_y_min)/Envi_y_unit * 1.0;
    double z_FloatIndex = (Point.z - Envi_z_min)/Envi_z_unit * 1.0;

    int x_leftindex = std::floor(x_FloatIndex);
    int y_leftindex = std::floor(y_FloatIndex);
    int z_leftindex = std::floor(z_FloatIndex);

    if(x_leftindex<0) {
      x_leftindex = 0;
    } else {
      if(x_leftindex>GridNo-2) {
        x_leftindex = GridNo-2;
      }
    }

    if(y_leftindex<0) {
      y_leftindex = 0;
    } else {
      if(y_leftindex>GridNo-2) {
        y_leftindex = GridNo-2;
      }
    }

    if(z_leftindex<0) {
      z_leftindex = 0;
    } else {
      if(z_leftindex>GridNo-2) {
        z_leftindex = GridNo-2;
      }
    }

    int x_rightindex = x_leftindex + 1;
    int y_rightindex = y_leftindex + 1;
    int z_rightindex = z_leftindex + 1;

    double valA = SDFTensor(x_leftindex, y_leftindex, z_leftindex);
    double valB = SDFTensor(x_rightindex, y_leftindex, z_leftindex);
    double valC = SDFTensor(x_rightindex, y_rightindex, z_leftindex);
    double valD = SDFTensor(x_leftindex, y_rightindex, z_leftindex);

    double valE = SDFTensor(x_leftindex, y_leftindex, z_rightindex);
    double valF = SDFTensor(x_rightindex, y_leftindex, z_rightindex);
    double valG = SDFTensor(x_rightindex, y_rightindex, z_rightindex);
    double valH = SDFTensor(x_leftindex, y_rightindex, z_rightindex);

    // Since this is a tri-linear interpolation, there are three ways to do the interpolation

    /*
      Jacobian matrix in the x-direction
    */

    // Cut the point with a plane orthgonal to z axis
    double valMAE =  (z_FloatIndex - z_leftindex*1.0) * (valE - valA) + valA;
    double valMBF =  (z_FloatIndex - z_leftindex*1.0) * (valF - valB) + valB;
    double valMDH =  (z_FloatIndex - z_leftindex*1.0) * (valH - valD) + valD;
    double valMCG =  (z_FloatIndex - z_leftindex*1.0) * (valG - valC) + valC;
    // Cut the point with a plane orthgonal to y axis
    double valMAEDH = (y_FloatIndex - y_leftindex*1.0) * (valMDH - valMAE) + valMAE;
    double valMBFCG = (y_FloatIndex - y_leftindex*1.0) * (valMCG - valMBF) + valMBF;
    // The values at the edge give the jacobian to x
    double JacDistTo_x = (valMBFCG - valMAEDH)/Envi_x_unit;

    /*
      Jacobian matrix in the y-direction
    */
    double valMABEF = (x_FloatIndex - x_leftindex*1.0) * (valMBF - valMAE) + valMAE;
    double valMDHCG = (x_FloatIndex - x_leftindex*1.0) * (valMCG - valMDH) + valMDH;

    double JacDistTo_y = (valMDHCG - valMABEF)/Envi_y_unit;

    /*
      Jacobian matrix in the z-direction
    */
    // Cut the point with a plane orthgonal to x axis
    double valMAB = (x_FloatIndex - x_leftindex*1.0) * (valB - valA) + valA;
    double valMDC = (x_FloatIndex - x_leftindex*1.0) * (valC - valD) + valD;
    double valMEF = (x_FloatIndex - x_leftindex*1.0) * (valF - valE) + valE;
    double valMHG = (x_FloatIndex - x_leftindex*1.0) * (valG - valH) + valH;
    // Cut the point with a plane orthgonal to y axis
    double valMABDC = (y_FloatIndex - y_leftindex*1.0) * (valMDC - valMAB) + valMAB;
    double valMEFHG = (y_FloatIndex - y_leftindex*1.0) * (valMHG - valMEF) + valMHG;
    // The values at the edge give the jacobian to z
    double JacDistTo_z = (valMEFHG - valMABDC)/Envi_z_unit;

    Vector3 RawNormal(JacDistTo_x, JacDistTo_y, JacDistTo_z);
    RawNormal.setNormalized(RawNormal);
    return RawNormal;
  }
  Eigen::Tensor<double, 3> SDFTensor;
  double Envi_x_min, Envi_x_max;
  double Envi_y_min, Envi_y_max;
  double Envi_z_min, Envi_z_max;
  double Envi_x_unit, Envi_y_unit, Envi_z_unit;
  double Envi_x_length, Envi_y_length, Envi_z_length;
  int GridNo;
};

struct ContactStatusInfo{
  ContactStatusInfo(){ LinkIndex = -1;}
  ContactStatusInfo(const int & link_index){ LinkIndex = link_index; }
  void AddLocalConactStatus(const int & _contactstatus){ LocalContactStatus.push_back(_contactstatus); }
  void StatusSwitch(const int & Val){
    for(int i = 0; i<LocalContactStatus.size(); i++)
      LocalContactStatus[i] = Val;
  }
  int LinkIndex;
  std::vector<int> LocalContactStatus;
};

struct PolarPoint {
  // This struct expresses a SO(3) point using Polar Coordinates with Radius and Direction
  PolarPoint(){
    Radius = -1.0;
    Position.setZero();
    Direction.setZero();
  };
  PolarPoint(double Radius_, const Vector3 & Position_) {
    // Constructor
    Radius = Radius_;
    Position = Position_;
    Direction = Position;
    double PositionLength = Position.norm();
    Direction.x = Direction.x/PositionLength;
    Direction.y = Direction.y/PositionLength;
    Direction.z = Direction.z/PositionLength;
  }
  double Radius;
  Vector3 Position;
  Vector3 Direction;
};

struct ReachabilityMap {
  // This struct is used to save the reachability map
  ReachabilityMap(){
    RadiusMin = -1.0;
    RadiusMax = -1.0;
    LayerSize = -1;
    RadiusUnit = -1.0;
    PointSize = -1;

  };
  ReachabilityMap(double RadiusMin_, double RadiusMax_, int LayerSize_){
    RadiusMin = RadiusMin_;
    RadiusMax = RadiusMax_;
    LayerSize = LayerSize_;
    RadiusUnit = (RadiusMax - RadiusMin)/(1.0 * LayerSize - 1.0);
    PointSize = -1;
  }
  std::vector<Vector3> ReachablePointsFinder(const Robot & SimRobot, int SwingLinkInfoIndex, const SDFInfo & SDFInfoObj) const{
    const double DisTol = 0.01;        // 1cm as a signed distance tolerance.

    double Radius           = EndEffectorChainRadius[SwingLinkInfoIndex];
    double PivotalLinkIndex = EndEffectorPivotalIndex[SwingLinkInfoIndex];
    Vector3 RefPoint;
    SimRobot.GetWorldPosition(Vector3(0.0, 0.0, 0.0), PivotalLinkIndex, RefPoint);
    
    std::vector<Vector3> ReachablePoints;
    ReachablePoints.reserve(PointSize);
    int LayerIndex = 0;
    double LayerRadius = RadiusMin;
    if(Radius>RadiusMax){
      LayerIndex = LayerSize-1;
    } else{
      while (LayerRadius<Radius){
        LayerRadius+=RadiusUnit;
        LayerIndex++;
      }
      LayerRadius-=RadiusUnit;
    }

    for (int i = 0; i < LayerIndex; i++){
      std::vector<PolarPoint> PolarPointLayer = PolarPointLayers.find(i)->second;
      for (int j = 0; j < PolarPointLayer.size(); j++){
        Vector3 PolarPointPos = PolarPointLayer[j].Position + RefPoint;
        double CurrentDist = SDFInfoObj.SignedDistance(PolarPointPos);
          if((CurrentDist>0.0)&&(CurrentDist<DisTol)){
          ReachablePoints.push_back(PolarPointPos);
        }
      }
    }
    return ReachablePoints;
  }

  std::vector<Vector3> ContactFreePointsFinder(double radius, const std::vector<Vector3> & ReachablePoints,
                                                              const std::vector<std::pair<Vector3, double>> & ContactFreeInfo) const{
    // This function can only be called after ReachablePointsFinder() to reduce the extra point further.
    std::vector<Vector3> ContactFreePoints;
    ContactFreePoints.reserve(ReachablePoints.size());
    int ContactFreeNo = 0;
    for (int i = 0; i < ReachablePoints.size(); i++){
      Vector3 ReachablePoint = ReachablePoints[i];
      bool ContactFreeFlag = true;
      int ContactFreeInfoIndex = 0;
      while (ContactFreeInfoIndex<ContactFreeInfo.size()){
        Vector3 RefPoint = ContactFreeInfo[ContactFreeInfoIndex].first;
        double Radius = ContactFreeInfo[ContactFreeInfoIndex].second;
        Vector3 PosDiff = ReachablePoint - RefPoint;
        double PosDiffDis = PosDiff.norm();
        if(PosDiffDis<=(Radius + radius)){
          ContactFreeFlag = false;
          break;
        }
        ContactFreeInfoIndex++;
      }
      if(ContactFreeFlag){
        ContactFreePoints.push_back(ReachablePoints[i]);
        ContactFreeNo++;
      }
    }
    return ContactFreePoints;
  };

  std::map<int, std::vector<PolarPoint>> PolarPointLayers;

  std::vector<double> EndEffectorChainRadius;                     // Radius from end effector to pivital joint
  std::vector<double> EndEffectorGeometryRadius;                  // End effector's geometrical radius
  std::vector<int>    EndEffectorLinkIndex;                       // End effector link index
  std::vector<int>    EndEffectorPivotalIndex;

  std::map<int, std::vector<int>> EndEffectorChainIndices;        // This map saves intermediate joint from End Effector Joint to Pivotal Joint.
  std::map<int, int>  EndEffectorIndexCheck;

  std::vector<int>    Link34ToPivotal;
  std::vector<int>    Link27ToPivotal;  

  double  RadiusMin;
  double  RadiusMax ;
  int     LayerSize;
  double  RadiusUnit;

  int     PointSize;
};

struct SimPara{
  SimPara(){
    PushDuration = -1.0;
    DetectionWait = -1.0;

    TimeStep = -1.0;
    InitDuration = -1.0;
    TotalDuration = -1.0;

    ForwardDurationSeed = -1.0;
    PhaseRatio = -1.0;  
    ReductionRatio = -1.0;

    ImpulseForceValue = -1.0;
    ImpulseForceDirection.setZero();
  };
  SimPara(const std::vector<double> & SimParaVec) {
    assert (SimParaVec.size() == 8);
    PushDuration = SimParaVec[0];
    DetectionWait = SimParaVec[1];

    TimeStep = SimParaVec[2];
    InitDuration = SimParaVec[3];
    TotalDuration = SimParaVec[4];

    ForwardDurationSeed = SimParaVec[5];
    PhaseRatio = SimParaVec[6];             // This ratio determines the boundary between acceleration and deceleration.
    ReductionRatio = SimParaVec[7];
  }
  // void CurrentCasePathUpdate(const string _CurrentCasePath){
  //   CurrentCasePath = _CurrentCasePath;

  //   string fedge_aFile = CurrentCasePath + "EdgeATraj.txt";
  //   // const char *fedge_aFile_Name = fedge_aFile.c_str();
  //   string fedge_bFile = CurrentCasePath + "EdgeBTraj.txt";
  //   // const char *fedge_bFile_Name = fedge_bFile.c_str();
  //   string fEdgeCOMFile = CurrentCasePath + "EdgeCOMTraj.txt";
  //   // const char *fEdgeCOMFile_Name = fEdgeCOMFile.c_str();
  //   string fEdgexTrajFile = CurrentCasePath + "EdgexTraj.txt";
  //   // const char *fEdgexTrajFile_Name = fEdgexTrajFile.c_str();
  //   string fEdgeyTrajFile = CurrentCasePath + "EdgeyTraj.txt";
  //   // const char *fEdgeyTrajFile_Name = fEdgeyTrajFile.c_str();
  //   string fEdgezTrajFile = CurrentCasePath + "EdgezTraj.txt";
  //   // const char *fEdgezTrajFile_Name = fEdgezTrajFile.c_str();
  //   string fVertexTrajFile = CurrentCasePath + "EdgeVertexTraj.txt";

  //   EdgeFileNames.clear();

  //   EdgeFileNames.push_back(fedge_aFile);
  //   EdgeFileNames.push_back(fedge_bFile);
  //   EdgeFileNames.push_back(fEdgeCOMFile);
  //   EdgeFileNames.push_back(fEdgexTrajFile);
  //   EdgeFileNames.push_back(fEdgeyTrajFile);
  //   EdgeFileNames.push_back(fEdgezTrajFile);
  //   EdgeFileNames.push_back(fVertexTrajFile);

  //   FailureStateTrajStr =  CurrentCasePath + "FailureStateTraj.path";
  //   // const char *FailureStateTrajStr_Name = FailureStateTrajStr.c_str();
  //   CtrlStateTrajStr    =  CurrentCasePath + "CtrlStateTraj.path";
  //   // const char *CtrlStateTrajStr_Name = CtrlStateTrajStr.c_str();
  //   PlanStateTrajStr = CurrentCasePath + "PlanStateTraj.path";
  //   // const char *PlanStateTrajStr_Name = PlanStateTrajStr.c_str();
    
  //   CtrlCFTrajStr = CurrentCasePath + "CtrlCFTraj.txt";
  //   FailureCFTrajStr = CurrentCasePath + "FailureCFTraj.txt";

  //   CtrlVelTrajStr = CurrentCasePath + "CtrlVelTraj.txt";
  //   FailureVelTrajStr = CurrentCasePath + "FailureVelTraj.txt";

  //   CtrlKETrajStr = CurrentCasePath + "CtrlKETraj.txt";
  //   FailureKETrajStr = CurrentCasePath + "FailureKETraj.txt";
  // }
  void setImpulseForce(double ImpulseForceValue_, Vector3 ImpulseForceDirection_){ ImpulseForceValue = ImpulseForceValue_; ImpulseForceDirection = ImpulseForceDirection_;}
  void getImpulseForce(double & ImpulseForceValue_, Vector3 & ImpulseForceDirection_ ) const { ImpulseForceValue_ = ImpulseForceValue;  ImpulseForceDirection_ = ImpulseForceDirection; }
  // string getCurrentCasePath() const{ return CurrentCasePath; }
  // void setImpulseForceMax(const Vector3 & ImpulseDirection){ ImpulseForceMax = ForceMax * ImpulseDirection; }
  // void setPlanStageIndex(const int & _PlanStageIndex) {PlanStageIndex = _PlanStageIndex; }
  // int  getPlanStageIndex() const{ return PlanStageIndex; }
  // void setPlanEndEffectorIndex( const int & _PlanEndEffectorIndex) { PlanEndEffectorIndex = _PlanEndEffectorIndex; }
  // int  getPlanEndEffectorIndex() const{ return PlanEndEffectorIndex; }
  // void setSimTime(const double & _SimTime) { SimTime = _SimTime; }
  // double getSimTime() const{ return SimTime; }
  // void setContactInit(const Vector3 _ContactInit){ ContactInit = _ContactInit; }
  // Vector3 getContactInit() const{ return ContactInit; }
  // void setContactGoal(const Vector3 _ContactGoal){ ContactGoal = _ContactGoal;}
  // Vector3 getContactGoal() const{ return ContactGoal;}
  // void setDirectionInit(const Vector3 & _DirectionInit ){ DirectionInit = _DirectionInit; }
  // void setDirectionGoal(const Vector3 & _DirectionGoal ){ DirectionGoal = _DirectionGoal; }
  // Vector3 getGoalDirection() const{ return DirectionGoal; }
  // void setTransPathFeasiFlag(const bool & _TransPathFeasiFlag){ TransPathFeasiFlag = _TransPathFeasiFlag; }
  // bool getTransPathFeasiFlag() const{ return TransPathFeasiFlag;}
  // void setSwingLinkInfoIndex(const int _SwingLinkInfoIndex) {SwingLinkInfoIndex = _SwingLinkInfoIndex; }
  // int  getSwingLinkInfoIndex() const{ return SwingLinkInfoIndex;}
  // void setCurrentContactPos(const Vector3 & _CurrentContactPos) {CurrentContactPos = _CurrentContactPos; }
  // Vector3 getCurrentContactPos() const{ return CurrentContactPos; }
  // void setTrajConfigOptFlag(const bool & _TrajConfigOptFlag){ TrajConfigOptFlag = _TrajConfigOptFlag;}
  // bool getTrajConfigOptFlag() const{return TrajConfigOptFlag;}
  // void setFixedContactStatusInfo(const std::vector<ContactStatusInfo> & _FixedContactStatusInfo){ FixedContactStatusInfo =_FixedContactStatusInfo;}

  double  PushDuration;
  double  DetectionWait;

  double  TimeStep;
  double  InitDuration;
  double  TotalDuration;

  double  ForwardDurationSeed;
  double  PhaseRatio;             // This ratio determines the boundary between acceleration and deceleration.
  double  ReductionRatio;

  double  ImpulseForceValue;
  Vector3 ImpulseForceDirection;

  // int     PlanStageIndex;
  // int     PlanEndEffectorIndex;    // This PlanEndEffectorIndex saves successful end effector for push recovery.
  // double  SimTime;
  // double  FailureTime;
  // bool    TransPathFeasiFlag;
  // int     SwingLinkInfoIndex;
  // Vector3 CurrentContactPos;
  // bool    TrajConfigOptFlag;

  // DataRecorderInfo DataRecorderObj;
  // std::string CurrentCasePath;
  // std::vector<string> EdgeFileNames;
  // string CtrlCFTrajStr, FailureCFTrajStr;
  // string CtrlKETrajStr, FailureKETrajStr;
  // string CtrlVelTrajStr, FailureVelTrajStr;
  // string FailureStateTrajStr, CtrlStateTrajStr, PlanStateTrajStr;
  // Vector3 ContactInit, ContactGoal;
  // Vector3 DirectionInit, DirectionGoal;
  // Vector3 EndEffectorInitxDir, EndEffectorInityDir;   // For alignment purpose
  // std::vector<ContactStatusInfo> FixedContactStatusInfo;
};

struct SelfCollisionInfo{
  SelfCollisionInfo(){
    SelfLinkGeoFlag = false;
  };
  SelfCollisionInfo(const Robot & SimRobot, const std::map<int, std::vector<int>> & EndEffectorChainIndices, const std::vector<int> & SelfCollisionFreeLink){
    for (int i = 5; i < SimRobot.q.size(); i++){
      AABB3D AABB3D_i = SimRobot.geometry[i]->GetAABBTight();
      Frame3D LinkTransforms_i = SimRobot.links[i].T_World;
      BoundingBoxes.push_back(AABB3D_i);
      Transforms.push_back(LinkTransforms_i);
    }

    for (int i = 0; i < EndEffectorChainIndices.size(); i++){
      std::vector<int> EndEffectorChainIndex = EndEffectorChainIndices.at(i);
      for (int j = 5; j < SimRobot.q.size(); j++){
        if(std::find(EndEffectorChainIndex.begin(), EndEffectorChainIndex.end(), j) == EndEffectorChainIndex.end()){
          if(std::find(SelfCollisionFreeLink.begin(), SelfCollisionFreeLink.end(), j) == SelfCollisionFreeLink.end()){
            SelfCollisionLinkMap[i].push_back(j-5);
          }
        }
      }
    }
    SelfLinkGeoFlag = true;
  };
  void SelfCollisionBoundingBoxesUpdate(const Robot& SimRobot){
    for (int i = 5; i < SimRobot.q.size(); i++){
      AABB3D AABB3D_i = SimRobot.geometry[i]->GetAABBTight();
      BoundingBoxes[i-5] = AABB3D_i;
      Frame3D LinkTransforms_i = SimRobot.links[i].T_World;
      Transforms[i-5] = LinkTransforms_i;
    }
  }

  void getSingleLinkDistNGrad(const int & LinkCountIndex, const Vector3 & GlobalPoint, double & Dist, Vector3 & DistGrad) const{
    const int GridNo = 100;
    double dx = BoundingBoxes[LinkCountIndex].size().x/(1.0 * GridNo);
    double dy = BoundingBoxes[LinkCountIndex].size().y/(1.0 * GridNo);
    double dz = BoundingBoxes[LinkCountIndex].size().z/(1.0 * GridNo);

    Dist = BoundingBoxes[LinkCountIndex].signedDistance(GlobalPoint);

    Vector3 GlobalPointx = GlobalPoint;
    GlobalPointx.x += dx;
    double Distx = BoundingBoxes[LinkCountIndex].signedDistance(GlobalPointx);

    Vector3 GlobalPointy = GlobalPoint;
    GlobalPointy.y += dy;
    double Disty = BoundingBoxes[LinkCountIndex].signedDistance(GlobalPointy);

    Vector3 GlobalPointz = GlobalPoint;
    GlobalPointz.z += dz;
    double Distz = BoundingBoxes[LinkCountIndex].signedDistance(GlobalPointz);

    DistGrad.x = (Distx - Dist)/dx;
    DistGrad.y = (Disty - Dist)/dy;
    DistGrad.z = (Distz - Dist)/dz;
    DistGrad.getNormalized(DistGrad);
  }

  double getSelfCollisionDist(const int & LinkIndex, const Vector3 & GlobalPoint) const{
    std::vector<int> SelfCollisionLinkIndices = SelfCollisionLinkMap.at(LinkIndex);
    const int ActLinkNo = SelfCollisionLinkIndices.size();
    std::vector<double> DistVec;
    DistVec.reserve(ActLinkNo);
    for (int i = 0; i < ActLinkNo; i++){
      int SelfCollisionLinkIndex = SelfCollisionLinkIndices[i];
      double Dist_i = BoundingBoxes[SelfCollisionLinkIndex].signedDistance(GlobalPoint);
      DistVec.push_back(Dist_i);
    }
    double Dist = *std::min_element(DistVec.begin(), DistVec.end());
    return Dist;
  }

  void getSelfCollisionDistNGrad(const int & LinkIndex, const Vector3 & GlobalPoint, double & Dist, Vector3 & Grad) const{
    // This function is used to calculate robot's self-collision distance given a point.
    std::vector<int> SelfCollisionLinkIndices = SelfCollisionLinkMap.at(LinkIndex);
    const int ActLinkNo = SelfCollisionLinkIndices.size();    
    std::vector<double> DistVec;
    DistVec.reserve(ActLinkNo);
    std::vector<Vector3> GradVec;
    GradVec.reserve(ActLinkNo);
    std::vector<double> DistWeights;
    DistWeights.reserve(ActLinkNo);
    double Dist_i;
    Vector3 Grad_i;
    for (int i = 0; i < ActLinkNo; i++){
      int SelfLinkIndex = SelfCollisionLinkIndices[i];
      getSingleLinkDistNGrad(SelfLinkIndex, GlobalPoint, Dist_i, Grad_i);
      DistVec.push_back(Dist_i);
      GradVec.push_back(Grad_i);
    }
    Dist = *std::min_element(DistVec.begin(), DistVec.end());
    double Scale = abs(Dist);
    for (int i = 0; i < ActLinkNo; i++){
      DistWeights.push_back(exp(-1.0 * DistVec[i]/Scale));
    }
    // Set its value to be zero!
    Grad.x = 0.0;
    Grad.y = 0.0;
    Grad.z = 0.0;
    for (int i = 0; i < ActLinkNo; i++)
      Grad+=DistWeights[i] * GradVec[i];
    Grad.getNormalized(Grad);
  }
  bool SelfLinkGeoFlag;
  std::vector<AABB3D> BoundingBoxes;
  std::vector<RigidTransform> Transforms;
  std::map<int, std::vector<int>> SelfCollisionLinkMap;       // This map saves intermediate joint from End Effector Joint to Pivotal Joint.
};

#endif
