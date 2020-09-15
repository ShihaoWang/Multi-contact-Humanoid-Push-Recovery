#ifndef ROBOTINFO_H
#define ROBOTINFO_H
#include <KrisLibrary/robotics/RobotDynamics3D.h>
#include <KrisLibrary/robotics/NewtonEuler.h>
#include <Interface/SimulationGUI.h>
#include <unsupported/Eigen/CXX11/Tensor>
#include <KrisLibrary/meshing/TriMeshTopology.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include "KrisLibrary/math3d/AABB3D.h"
#include "KrisLibrary/math3d/LocalCoordinates3D.h"
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

struct DataRecorderInfo{
  DataRecorderInfo(){
    PlanStageIndex = -1;
    LinkNo = -1;
    FailureMetric = 0.0;
  };
  void setPlanStageIndexNLinkNo(const int & _PlanStageIndex, const int & _LinkNo){
    PlanStageIndex = _PlanStageIndex;
    LinkNo = _LinkNo;
  }
  void setRCSData( const std::vector<Vector3> & _ReachableContacts,
                    const std::vector<Vector3> & _CollisionFreeContacts,
                    const std::vector<Vector3> & _SupportiveContacts){
                      ReachableContacts = _ReachableContacts;
                      CollisionFreeContacts = _CollisionFreeContacts;
                      SupportiveContacts = _SupportiveContacts;
  }
  void setCCSData(  const std::vector<Vector3> & _CandidateContacts,
                    const std::vector<Vector3> & _CandidateContactWeights,
                    const std::vector<Vector3> & _SelectedContacts){
                      CandidateContacts = _CandidateContacts;
                      CandidateContactWeights = _CandidateContactWeights;
                      SelectedContacts = _SelectedContacts;
  }
  void setPathWaypoints(const std::vector<Vector3> & _PathWaypoints){ PathWaypoints = _PathWaypoints; }
  void setTrajs(const LinearPath & _PlannedConfigTraj, const LinearPath & _EndEffectorTraj){
    PlannedConfigTraj = _PlannedConfigTraj;
    EndEffectorTraj = _EndEffectorTraj;
  }
  void Vector3Writer(const std::vector<Vector3> & ContactPoints, const std::string & ContactPointFileName){
    if(ContactPoints.size() ==0) return;
    int NumberOfContactPoints = ContactPoints.size();
    std::vector<double> FlatContactPoints(3 * NumberOfContactPoints);
    int FlatContactPointIndex = 0;
    for (int i = 0; i < NumberOfContactPoints; i++){
      FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].x;
      FlatContactPointIndex++;
      FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].y;
      FlatContactPointIndex++;
      FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].z;
      FlatContactPointIndex++;
    }
    FILE * FlatContactPointsFile = NULL;
    string ContactPointFile = ContactPointFileName + ".bin";
    const char *ContactPointFile_Name = ContactPointFile.c_str();
    FlatContactPointsFile = fopen(ContactPointFile_Name, "wb");
    fwrite(&FlatContactPoints[0], sizeof(double), FlatContactPoints.size(), FlatContactPointsFile);
    fclose(FlatContactPointsFile);
    return;
  }
  void UpdateWithRecoveryReferenceInfo(const LinearPath & PlannedConfigTraj_, const LinearPath & EndEffectorTraj_, double FailureMetric_){
    PlannedConfigTraj = PlannedConfigTraj_;
    EndEffectorTraj   = EndEffectorTraj_;
    FailureMetric     = FailureMetric_;
  }
  void Write2File(const string & CurrentCasePath){
    // This function will only be called if planning is successful!
    const string InnerPath = CurrentCasePath + std::to_string(PlanStageIndex) + "_" + std::to_string(LinkNo) + "_";
    Vector3Writer(ReachableContacts,      InnerPath + "ReachableContacts");
    Vector3Writer(CollisionFreeContacts,  InnerPath + "CollisionFreeContacts");
    Vector3Writer(SupportiveContacts,     InnerPath + "SupportiveContacts");
    Vector3Writer(CandidateContacts,      InnerPath + "CandidateContacts");
    Vector3Writer(CandidateContactWeights,InnerPath + "CandidateContactWeights");
    Vector3Writer(SelectedContacts,       InnerPath + "SelectedContacts");
    Vector3Writer(PathWaypoints,          InnerPath + "PathWaypoints");

    // Failure Metric for strategy selection
    const string FailureMetricFileName = InnerPath + "FailureMetric.txt";
    std::ofstream FailureMetricFile (FailureMetricFileName);
    FailureMetricFile << FailureMetric;
    FailureMetricFile.close();

    // Write these two trajectories into files.
    std::ofstream PlannedConfigTrajFile;
    const string PlannedConfigTrajName = InnerPath + "PlannedConfigTraj.path";
    PlannedConfigTrajFile.open (PlannedConfigTrajName.c_str());
    PlannedConfigTraj.Save(PlannedConfigTrajFile);
    PlannedConfigTrajFile.close();

    std::ofstream EndEffectorTrajFile;
    const string EndEffectorTrajName = InnerPath + "EndEffectorTraj.path";
    EndEffectorTrajFile.open(EndEffectorTrajName.c_str());
    EndEffectorTraj.Save(EndEffectorTrajFile);
    EndEffectorTrajFile.close();
  }
  std::vector<Vector3> ReachableContacts;
  std::vector<Vector3> CollisionFreeContacts;
  std::vector<Vector3> SupportiveContacts;
  std::vector<Vector3> CandidateContacts;
  std::vector<Vector3> CandidateContactWeights;
  std::vector<Vector3> SelectedContacts;
  std::vector<Vector3> PathWaypoints;

  LinearPath  PlannedConfigTraj;
  LinearPath  EndEffectorTraj;
  double      FailureMetric;

  int PlanStageIndex;
  int LinkNo;
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
    const double DisTol = 0.01;        // 1cm as a distance tolerance.

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

    ContactSelectionCoeff = 1.25;

    ImpulseForceValue = -1.0;
    ImpulseForceDirection.setZero();

    FailureTime = -1.0;

    SelfCollisionTol = -1.0; 
    TouchDownTol = -1.0;
  };
  SimPara(const std::vector<double> & SimParaVec) {
    assert (SimParaVec.size() == 11);
    PushDuration = SimParaVec[0];
    DetectionWait = SimParaVec[1];

    TimeStep = SimParaVec[2];
    InitDuration = SimParaVec[3];
    TotalDuration = SimParaVec[4];

    ForwardDurationSeed = SimParaVec[5];
    PhaseRatio = SimParaVec[6];             // This ratio determines the boundary between acceleration and deceleration.
    ReductionRatio = SimParaVec[7];

    ContactSelectionCoeff = SimParaVec[8];

    SelfCollisionTol = SimParaVec[9];
    TouchDownTol = SimParaVec[10];
  }
  void CurrentCasePathUpdate(const string & CurrentCasePath_){
    CurrentCasePath = CurrentCasePath_;

    string fedge_aFile = CurrentCasePath + "EdgeATraj.txt";
    // const char *fedge_aFile_Name = fedge_aFile.c_str();
    string fedge_bFile = CurrentCasePath + "EdgeBTraj.txt";
    // const char *fedge_bFile_Name = fedge_bFile.c_str();
    string fEdgeCOMFile = CurrentCasePath + "EdgeCOMTraj.txt";
    // const char *fEdgeCOMFile_Name = fEdgeCOMFile.c_str();
    string fEdgexTrajFile = CurrentCasePath + "EdgexTraj.txt";
    // const char *fEdgexTrajFile_Name = fEdgexTrajFile.c_str();
    string fEdgeyTrajFile = CurrentCasePath + "EdgeyTraj.txt";
    // const char *fEdgeyTrajFile_Name = fEdgeyTrajFile.c_str();
    string fEdgezTrajFile = CurrentCasePath + "EdgezTraj.txt";
    // const char *fEdgezTrajFile_Name = fEdgezTrajFile.c_str();
    string fVertexTrajFile = CurrentCasePath + "EdgeVertexTraj.txt";

    EdgeFileNames.clear();

    EdgeFileNames.push_back(fedge_aFile);
    EdgeFileNames.push_back(fedge_bFile);
    EdgeFileNames.push_back(fEdgeCOMFile);
    EdgeFileNames.push_back(fEdgexTrajFile);
    EdgeFileNames.push_back(fEdgeyTrajFile);
    EdgeFileNames.push_back(fEdgezTrajFile);
    EdgeFileNames.push_back(fVertexTrajFile);

    FailureStateTrajStr =  CurrentCasePath + "FailureStateTraj.path";
    // const char *FailureStateTrajStr_Name = FailureStateTrajStr.c_str();
    CtrlStateTrajStr    =  CurrentCasePath + "CtrlStateTraj.path";
    // const char *CtrlStateTrajStr_Name = CtrlStateTrajStr.c_str();
    PlanStateTrajStr = CurrentCasePath + "PlanStateTraj.path";
    // const char *PlanStateTrajStr_Name = PlanStateTrajStr.c_str();

    CtrlPosTrajStr = CurrentCasePath + "CtrlPosTraj.txt";
    FailurePosTrajStr = CurrentCasePath + "FailurePosTraj.txt";
    
    CtrlVelTrajStr = CurrentCasePath + "CtrlVelTraj.txt";
    FailureVelTrajStr = CurrentCasePath + "FailureVelTraj.txt";
  }

  void    setImpulseForce(double ImpulseForceValue_, Vector3 ImpulseForceDirection_){ ImpulseForceValue = ImpulseForceValue_; ImpulseForceDirection = ImpulseForceDirection_;}
  void    getImpulseForce(double & ImpulseForceValue_, Vector3 & ImpulseForceDirection_ ) const { ImpulseForceValue_ = ImpulseForceValue;  ImpulseForceDirection_ = ImpulseForceDirection; }
  string  getCurrentCasePath() const{ return CurrentCasePath; }
  void    setPlanStageIndex(const int & _PlanStageIndex) {PlanStageIndex = _PlanStageIndex; }
  int     getPlanStageIndex() const{ return PlanStageIndex; }
  void    setSimTime(const double & _SimTime) { SimTime = _SimTime; }
  double  getSimTime() const{ return SimTime; }
  void    setInitContactPos(const Vector3 & InitContactPos_){ InitContactPos = InitContactPos_; }
  Vector3 getInitContactPos() const{ return InitContactPos; }
  void    setInitContactDirection(const Vector3 & InitContactDirection_) { InitContactDirection = InitContactDirection_; }
  Vector3 getInitContactDirection() const{return InitContactDirection; }
  void    setGoalContactPos(const Vector3 & GoalContactPos_) { GoalContactPos = GoalContactPos_; }
  Vector3 getGoalContactPos() const{ return GoalContactPos; }
  void    setGoalContactDirection(const Vector3 & GoalContactDirection_) { GoalContactDirection = GoalContactDirection_; }
  Vector3 getGoalContactDirection() const{ return GoalContactDirection; };

  void    setSwingLinkInfoIndex(const int & _SwingLinkInfoIndex) { SwingLinkInfoIndex = _SwingLinkInfoIndex; }
  int     getSwingLinkInfoIndex() const{ return SwingLinkInfoIndex;}
  double  getContactSelectionCoeff() const{ return ContactSelectionCoeff; }

  void    setOneHandAlreadyFlag(const bool & OneHandAlreadyFlag_) { OneHandAlreadyFlag = OneHandAlreadyFlag_; }
  bool    getOneHandAlreadyFlag() const { return OneHandAlreadyFlag; }

  void    setPlanEndEffectorIndex( const int & PlanEndEffectorIndex_) { PlanEndEffectorIndex = PlanEndEffectorIndex_; }
  int     getPlanEndEffectorIndex() const{ return PlanEndEffectorIndex; }
  void    setFixedContactStatusInfo(const std::vector<ContactStatusInfo> & _FixedContactStatusInfo){ FixedContactStatusInfo =_FixedContactStatusInfo;}
  std::vector<ContactStatusInfo> getFixedContactStatusInfo() const { return FixedContactStatusInfo; }
  
  double  getSelfCollisionTol() const { return SelfCollisionTol; }
  double  getTouchDownTol() const { return TouchDownTol; }

  // void    setTransPathFeasiFlag(const bool & _TransPathFeasiFlag){ TransPathFeasiFlag = _TransPathFeasiFlag; }
  // bool    getTransPathFeasiFlag() const{ return TransPathFeasiFlag; }

  // void setCurrentContactPos(const Vector3 & _CurrentContactPos) {CurrentContactPos = _CurrentContactPos; }
  // Vector3 getCurrentContactPos() const{ return CurrentContactPos; }
  // void setTrajConfigOptFlag(const bool & _TrajConfigOptFlag){ TrajConfigOptFlag = _TrajConfigOptFlag;}
  // bool getTrajConfigOptFlag() const{return TrajConfigOptFlag;}

  double  PushDuration;
  double  DetectionWait;

  double  TimeStep;
  double  InitDuration;
  double  TotalDuration;

  double  ForwardDurationSeed;
  double  PhaseRatio;             // This ratio determines the boundary between acceleration and deceleration.
  double  ReductionRatio;

  double  ContactSelectionCoeff; 

  double  SelfCollisionTol;       // 1.0cm
  double  TouchDownTol;           // 1.0cm
  
  double  ImpulseForceValue;
  Vector3 ImpulseForceDirection;

  string  CurrentCasePath;
  std::vector<string> EdgeFileNames;
  string  FailureStateTrajStr, CtrlStateTrajStr, PlanStateTrajStr;
  string  CtrlPosTrajStr, FailurePosTrajStr;
  string  CtrlVelTrajStr, FailureVelTrajStr;

  double  FailureTime;
  int     PlanStageIndex;
  double  SimTime;

  int     SwingLinkInfoIndex;
  bool    OneHandAlreadyFlag; 
  int     PlanEndEffectorIndex;     // This PlanEndEffectorIndex saves successful end effector for push recovery.
  // Vector3 CurrentContactPos;
  // bool    TrajConfigOptFlag;

  DataRecorderInfo DataRecorderObj;
  Vector3 InitContactPos, InitContactDirection;
  Vector3 GoalContactPos, GoalContactDirection;
  std::vector<ContactStatusInfo> FixedContactStatusInfo;

  // Vector3 EndEffectorInitxDir, EndEffectorInityDir;   // For alignment purpose
};

struct SelfCollisionInfo: public ScaledLocalCoordinates3D{
  SelfCollisionInfo(){
    SelfLinkGeoFlag = false;
  };
  SelfCollisionInfo(const Robot & SimRobot, const std::map<int, std::vector<int>> & EndEffectorChainIndices, const std::vector<int> & SelfCollisionFreeLink){
    for (int i = 5; i < SimRobot.q.size(); i++){
      Box3D Box3D_i = SimRobot.geometry[i]->GetBB();
      Frame3D LinkTransforms_i = SimRobot.links[i].T_World;
      BoundingBoxes.push_back(Box3D_i);
      Vector3 Extremity_i;
      RigidTransform Transformation_i;
      BoxInfoUpdate(Box3D_i, Extremity_i, Transformation_i);
      BoundingBoxExtremities.push_back(Extremity_i);
      BoundingBoxTransforms.push_back(Transformation_i);
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
      Box3D Box3D_i = SimRobot.geometry[i]->GetBB();
      BoundingBoxes[i-5] = Box3D_i;
      Vector3 Extremity_i;
      RigidTransform Transformation_i;
      BoxInfoUpdate(Box3D_i, Extremity_i, Transformation_i);
      BoundingBoxExtremities[i-5] = Extremity_i;
      BoundingBoxTransforms[i-5] = Transformation_i;
    }
  }

  void BBVerticesWriter() const {
    for (int i = 0; i < BoundingBoxes.size(); i++){
      std::vector<Vector3> BoundingBoxVertices = BBVertices(i);
      string BBName = "BB" + to_string(i); 
      Vector3Writer(BoundingBoxVertices, BBName);
    }
  }

  void BoxInfoUpdate(const Box3D & Box, Vector3 & Extremity_, RigidTransform & Transformation_){
    // This function is used to update the bounding box information.
    double x_scale = Box.dims.x;
    double y_scale = Box.dims.y;
    double z_scale = Box.dims.z;

    // Get rid of the fact that the axes have been scaled
    Vector3 xbasis = Box.xbasis;
    Vector3 ybasis = Box.ybasis;
    Vector3 zbasis = Box.zbasis;

    Vector3 xbasis_, ybasis_, zbasis_;
    xbasis_.setNormalized(xbasis);
    ybasis_.setNormalized(ybasis);
    zbasis_.setNormalized(zbasis);

    Vector3 origin = Box.origin;

    double x_mag = xbasis.norm();
    double y_mag = ybasis.norm();
    double z_mag = zbasis.norm();

    double x_scale_act = x_scale * x_mag;
    double y_scale_act = y_scale * y_mag;
    double z_scale_act = z_scale * z_mag;
    
    Vector3 Extremity(x_scale_act, y_scale_act, z_scale_act);
    Extremity_ = Extremity;
    RigidTransform Transformation(xbasis_, ybasis_, zbasis_, origin);
    Transformation_ = Transformation;
  }

  std::vector<Vector3> BBVertices(int BBIndex) const{
    std::vector<Vector3> Vertices(8);
    // This function calculates the vertices for current bounding box.
    Box3D CurBB = BoundingBoxes[BBIndex];

    double x_scale = CurBB.dims.x;
    double y_scale = CurBB.dims.y;
    double z_scale = CurBB.dims.z;

    Vector3 p0(0.0,      0.0,       0.0);
    Vector3 p1(x_scale,  0.0,       0.0);
    Vector3 p2(0.0,      y_scale,   0.0);
    Vector3 p3(x_scale,  y_scale,   0.0);

    Vector3 pz(0.0,      0.0,      z_scale);

    Vector3 p4 = p0 + pz;
    Vector3 p5 = p1 + pz;
    Vector3 p6 = p2 + pz;
    Vector3 p7 = p3 + pz;

    Vector3 p0_, p1_, p2_, p3_, p4_, p5_, p6_, p7_;

	  Matrix4 basis;
	  CurBB.getBasis(basis);
    basis.mulPoint(p0, p0_);
    basis.mulPoint(p1, p1_);
    basis.mulPoint(p2, p2_);
    basis.mulPoint(p3, p3_);
    basis.mulPoint(p4, p4_);
    basis.mulPoint(p5, p5_);
    basis.mulPoint(p6, p6_);
    basis.mulPoint(p7, p7_);

    Vertices[0] = p0_;
    Vertices[1] = p1_;
    Vertices[2] = p2_;
    Vertices[3] = p3_;
    Vertices[4] = p4_;
    Vertices[5] = p5_;
    Vertices[6] = p6_;
    Vertices[7] = p7_;

    return Vertices;
  }

  std::vector<double> LinearSpace(double a, double b, std::size_t N) const {
    double h = (b - a) / static_cast<double>(N-1);
    std::vector<double> xs(N);
    std::vector<double>::iterator x;
    double val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
      *x = val;
    }
    return xs;
  }

  std::vector<Vector3> TwoPointLine(const Vector3 & p1, const Vector3 & p2) const{
    int point_number = 100;
    std::vector<double> line_x = LinearSpace(p1.x, p2.x, point_number);
    std::vector<double> line_y = LinearSpace(p1.y, p2.y, point_number);
    std::vector<double> line_z = LinearSpace(p1.z, p2.z, point_number);
    std::vector<Vector3> LinePoints;
    Vector3 TempPoint;
    for (int i = 0; i < point_number; i++){
      TempPoint.x = line_x[i];
      TempPoint.y = line_y[i];
      TempPoint.z = line_z[i];
      LinePoints.push_back(TempPoint);
    }
    return LinePoints;
  }

  double Box3DsignedDistance(int BoxIndex, const Point3D & GlobalPoint) const{
    Box3D Box = BoundingBoxes[BoxIndex];
    Vector3 LocalPoint;
    BoundingBoxTransforms[BoxIndex].mulInverse(GlobalPoint, LocalPoint);
    Vector3 Extremity = BoundingBoxExtremities[BoxIndex];
    bool Inside = true;
    Vector3 LocalNearestPoint;
    // Comparison from three dimensions
    // x dimension
    if(LocalPoint.x < 0) { 
      LocalNearestPoint.x = 0.0; 
      Inside = false;
    } else {
      if(LocalPoint.x > Extremity.x) { 
        LocalNearestPoint.x = Extremity.x; 
        Inside = false;
      } else {
        LocalNearestPoint.x = LocalPoint.x;
      }
    }
    // y dimension
    if(LocalPoint.y < 0) { 
      LocalNearestPoint.y = 0.0; 
      Inside = false;
    } else {
      if(LocalPoint.y > Extremity.y) { 
        LocalNearestPoint.y = Extremity.y; 
        Inside = false;
      } else {
        LocalNearestPoint.y = LocalPoint.y;
      }
    }
    // z dimension
    if(LocalPoint.z < 0) { 
      LocalNearestPoint.z = 0.0; 
      Inside = false;
    } else {
      if(LocalPoint.z > Extremity.z) { 
        LocalNearestPoint.z = Extremity.z; 
        Inside = false;
      } else {
        LocalNearestPoint.z = LocalPoint.z;
      }
    }
    Vector3 GlobalNearestPoint;
    BoundingBoxTransforms[BoxIndex].mulPoint(LocalNearestPoint, GlobalNearestPoint);
    Vector3 PointDiff = GlobalNearestPoint - GlobalPoint;
    if(!Inside) return PointDiff.norm();
    else {
      double dmin = Inf;
      double leftbound = LocalPoint.x;
      double rightbound = Extremity.x - LocalPoint.x;
      dmin = min(dmin, min(leftbound, rightbound));    

      leftbound = LocalPoint.y;
      rightbound = Extremity.y - LocalPoint.y;
      dmin = min(dmin, min(leftbound, rightbound));

      leftbound = LocalPoint.z;
      rightbound = Extremity.z - LocalPoint.z;
      dmin = min(dmin, min(leftbound, rightbound));
      return -dmin;
    }
  }

  void Vector3Writer(const std::vector<Vector3> & ContactPoints, const std::string &ContactPointFileName) const{
    if(ContactPoints.size()==0){
      std::cerr<< " ContactPoints has zero element!\n" << endl;
    }
    int NumberOfContactPoints = ContactPoints.size();
    std::vector<double> FlatContactPoints(3 * NumberOfContactPoints);
    int FlatContactPointIndex = 0;
    for (int i = 0; i < NumberOfContactPoints; i++){
      FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].x;
      FlatContactPointIndex++;
      FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].y;
      FlatContactPointIndex++;
      FlatContactPoints[FlatContactPointIndex] = ContactPoints[i].z;
      FlatContactPointIndex++;
    }
    FILE * FlatContactPointsFile = NULL;
    string ContactPointFile = ContactPointFileName + ".bin";
    const char *ContactPointFile_Name = ContactPointFile.c_str();
    FlatContactPointsFile = fopen(ContactPointFile_Name, "wb");
    fwrite(&FlatContactPoints[0], sizeof(double), FlatContactPoints.size(), FlatContactPointsFile);
    fclose(FlatContactPointsFile);
    return;
  }

  void getSingleLinkDistNGrad(const int & LinkCountIndex, const Vector3 & GlobalPoint, double & Dist, Vector3 & DistGrad) const{
    const int GridNo = 100;
    double dx = BoundingBoxes[LinkCountIndex].dims.x/(1.0 * GridNo);
    double dy = BoundingBoxes[LinkCountIndex].dims.y/(1.0 * GridNo);
    double dz = BoundingBoxes[LinkCountIndex].dims.z/(1.0 * GridNo);

    Dist = Box3DsignedDistance(LinkCountIndex, GlobalPoint);

    Vector3 GlobalPointx = GlobalPoint;
    GlobalPointx.x += dx;
    double Distx = Box3DsignedDistance(LinkCountIndex, GlobalPointx);

    Vector3 GlobalPointy = GlobalPoint;
    GlobalPointy.y += dy;
    double Disty = Box3DsignedDistance(LinkCountIndex, GlobalPointy);

    Vector3 GlobalPointz = GlobalPoint;
    GlobalPointz.z += dz;
    double Distz = Box3DsignedDistance(LinkCountIndex, GlobalPointz);

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
      double Dist_i = Box3DsignedDistance(i, GlobalPoint);
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
  std::vector<Box3D>            BoundingBoxes;
  std::vector<Vector3>          BoundingBoxExtremities;               // Local Extremities
  std::vector<RigidTransform>   BoundingBoxTransforms;                // Local Transformation
  std::map<int, std::vector<int>> SelfCollisionLinkMap;               // This map saves intermediate joint from End Effector Joint to Pivotal Joint.
};

struct RecoveryReferenceInfo{
  RecoveryReferenceInfo(){
    ReadyFlag = false;
    TouchDownFlag = false;
    OneHandAlreadyFlag = false;
    ControlReferenceType = -1;
    SwingLinkInfoIndex = -1;           // Used for RobotLinkInfo
    ContactStatusInfoIndex = -1;
    WaitTime = -1.0;
    FailureMetric = -1.0;

    GoalContactPos.setZero();
    GoalContactGrad.setZero();
  }
  void setFailureMetric(const double & _FailureMetric){ FailureMetric = _FailureMetric; }
  double getFailureMetric() const{ return FailureMetric; }
  void setSwingLinkInfoIndex(const int & _SwingLinkInfoIndex) {SwingLinkInfoIndex = _SwingLinkInfoIndex;}
  int  getSwingLinkInfoIndex() const{ return SwingLinkInfoIndex; }
  bool getReadyFlag() const{ return ReadyFlag;}
  void setReadyFlag(const bool & _ReadyFlag ){ ReadyFlag = _ReadyFlag; }
  void setTouchDownFlag(const bool & _TouchDownFlag){ TouchDownFlag=_TouchDownFlag; }
  bool getTouchDownFlag(){ return TouchDownFlag; }
  void setOneHandAlreadyFlag(bool Value) {OneHandAlreadyFlag = Value;}
  bool getOneHandAlreadyFlag() const { return OneHandAlreadyFlag; };
  int  getControlReferenceType() const{ return ControlReferenceType; }
  void setControlReferenceType(const int &_ControlReferenceType) { ControlReferenceType = _ControlReferenceType; }
  void setGoalContactPosNGrad(const Vector3 & _GoalContactPos, const Vector3 & _GoalContactGrad){
    GoalContactPos = _GoalContactPos;
    GoalContactGrad = _GoalContactGrad;
  }
  Vector3 getGoalContactPos() const{ return GoalContactPos; }
  Vector3 getGoalContactGrad() const{ return GoalContactGrad;}

  void SetInitContactStatus(const std::vector<ContactStatusInfo> &_InitContactStatus){ InitContactStatus = _InitContactStatus; }
  std::vector<ContactStatusInfo> getInitContactStatus() const { return InitContactStatus;}

  void SetGoalContactStatus(const std::vector<ContactStatusInfo> & _GoalContactStatus) { GoalContactStatus = _GoalContactStatus;}
  std::vector<ContactStatusInfo> getGoalContactStatus() const { return GoalContactStatus;}

  void TrajectoryUpdate(const std::vector<double> & timeTraj, const std::vector<Config> & configTraj, const std::vector<Config> & velocityTraj, const std::vector<Vector3> & endeffectorTraj){
    TimeTraj = timeTraj;
    ConfigTraj = configTraj;
    PlannedConfigTraj = LinearPath(timeTraj, configTraj);
    PlannedVelocityTraj = LinearPath(timeTraj, velocityTraj);
    std::vector<Vector> endeffectorPath;
    for (Vector3 EndEffectorPos: endeffectorTraj){
      Vector EndEffectorPosVec;
      EndEffectorPosVec.resize(3);
      EndEffectorPosVec[0] = EndEffectorPos[0];
      EndEffectorPosVec[1] = EndEffectorPos[1];
      EndEffectorPosVec[2] = EndEffectorPos[2];
      endeffectorPath.push_back(EndEffectorPosVec);
    }
    EndEffectorTraj = LinearPath(timeTraj, endeffectorPath);
  }

  void setWaitTime(const double & _WaitTime) { WaitTime = _WaitTime; }
  double getWaitTime() const { return WaitTime; }
  void setTouchDownConfig(const std::vector<double> _TouchDownConfig){ TouchDownConfig = _TouchDownConfig; }
  std::vector<double> getTouchDownConfig() const { return TouchDownConfig;}

  bool    ReadyFlag;
  bool    TouchDownFlag;
  bool    OneHandAlreadyFlag;   
  int     ControlReferenceType;
  int     SwingLinkInfoIndex;
  int     ContactStatusInfoIndex;
  double  WaitTime;
  double  FailureMetric;

  Vector3 GoalContactPos;
  Vector3 GoalContactGrad;

  std::vector<double> TouchDownConfig;

  LinearPath PlannedConfigTraj;
  LinearPath PlannedVelocityTraj;
  LinearPath EndEffectorTraj;

  std::vector<QuaternionRotation> OrientationQuat;
  
  std::vector<double> TimeTraj;
  std::vector<Config> ConfigTraj;

  std::vector<ContactStatusInfo> InitContactStatus;
  std::vector<ContactStatusInfo> GoalContactStatus;
};

struct FacetInfo{
  FacetInfo(){
    FacetValidFlag = false;
  };
  void setFacetValidFlag(const bool & _FacetValidFlag){FacetValidFlag = _FacetValidFlag;}
  bool getFacetValidFlag(){ return FacetValidFlag;}
  void setFacetEdges(const std::vector<std::pair<Vector3, Vector3>> & _FacetEdges) { FacetEdges = _FacetEdges; }
  void setFacetNorm(const Vector3& _FacetNorm){ FacetNorm = _FacetNorm;}
  double ProjPoint2EdgeDist(const Vector3& _Point){
    std::vector<double> ProjPoint2Edge_vec(EdgeNorms.size());
    Vector3 Vertex2Point = _Point - FacetEdges[0].first;
    double Point2Facet = Vertex2Point.dot(FacetNorm);
    Vector3 Facet2Point = Point2Facet * FacetNorm;
    for (int i = 0; i < EdgeNorms.size(); i++){
      Vertex2Point = _Point - FacetEdges[i].first;
      Vector3 Vertex2ProjPoint = Vertex2Point - Facet2Point;
      double ProjPoint2Edge_i = Vertex2ProjPoint.dot(EdgeNorms[i]);
      ProjPoint2Edge_vec[i] = ProjPoint2Edge_i;
    }
    return *min_element(ProjPoint2Edge_vec.begin(), ProjPoint2Edge_vec.end());
  }
  std::vector<double> ProjPoint2EdgeDistVec(const Vector3& _Point){
    std::vector<double> ProjPoint2Edge_vec(EdgeNorms.size());
    Vector3 Vertex2Point = _Point - FacetEdges[0].first;
    double Point2Facet = Vertex2Point.dot(FacetNorm);
    Vector3 Facet2Point = Point2Facet * FacetNorm;
    for (int i = 0; i < EdgeNorms.size(); i++){
      Vertex2Point = _Point - FacetEdges[i].first;
      Vector3 Vertex2ProjPoint = Vertex2Point - Facet2Point;
      double ProjPoint2Edge_i = Vertex2ProjPoint.dot(EdgeNorms[i]);
      ProjPoint2Edge_vec[i] = ProjPoint2Edge_i;
    }
    return ProjPoint2Edge_vec;
  }
  void EdgesUpdate(){
    Edges.reserve(EdgeNorms.size());
    EdgesDirection.reserve(EdgeNorms.size());
    for (int i = 0; i < EdgeNorms.size(); i++){
      Vector3 Edge_i = FacetEdges[i].second - FacetEdges[i].first;
      Edges.push_back(Edge_i);
      Vector3 Edge_i_normalized;
      Edge_i.getNormalized(Edge_i_normalized);
      EdgesDirection.push_back(Edge_i_normalized);
    }
  }
  std::vector<std::pair<Vector3, Vector3>> FacetEdges;
  std::vector<Vector3> EdgeNorms;
  Vector3 FacetNorm;
  std::vector<Vector3> Edges;
  std::vector<Vector3> EdgesDirection;
  bool FacetValidFlag;
};

struct PIPInfo{
  // This struct saves the information of the projected inverted pendulum from the CoM to the edge of convex polytope
  PIPInfo(){
    L = 0.25;         // The reference bound range is [0.25, 0.85]
    Ldot = 0.0;
    theta = 0.0;
    thetadot = 0.0;
    g = 9.81;
    g_angle = 0.0;
    speed = -1.0;
    onFlag = false;
  }
  PIPInfo(double _L, double _Ldot, double _theta, double _thetadot, double _g, double _g_angle){
    L = _L;
    Ldot = _Ldot;
    theta = _theta;
    thetadot = _thetadot;
    g = _g;
    g_angle = _g_angle;
  }
  void setPrimeUnits(const Vector3 & x_prime_unit_,const Vector3 & y_prime_unit_,const Vector3 & z_prime_unit_){
    x_prime_unit = x_prime_unit_;
    y_prime_unit = y_prime_unit_;
    z_prime_unit = z_prime_unit_;
  }
  void setUnits(const Vector3 & x_unit_,const Vector3 & y_unit_,const Vector3 & z_unit_){
    x_unit = x_unit_;
    y_unit = y_unit_;
    z_unit = z_unit_;
  }
  void setEdgeAnB(const Vector3 & edge_a_, const Vector3 & edge_b_){
    edge_a = edge_a_;
    edge_b = edge_b_;
  }
  void setIntersection(const Vector3 & intersection_){ intersection = intersection_;}
  void setSpeed(const double & _speed ) {speed = _speed;}
  double getSpeed() const{ return speed;}

  double  L, Ldot, theta, thetadot;
  double  g, g_angle;

  double  speed;                            // This value indicates the horizontal velocity.
  bool    onFlag;                           // Whether the origin is at intersection or not?!

  Vector3 x_prime_unit, y_prime_unit, z_prime_unit;
  Vector3 x_unit, y_unit, z_unit;
  Vector3 edge_a, edge_b;                     // The Edge points from edge_a to edge_b.
  Vector3 intersection;                     // The point where the COM intersects the edge.
};

struct ContactForm{
  ContactForm(){
    SwingLinkInfoIndex = -1;
    ContactType = -1;
  };
  ContactForm(      const std::vector<ContactStatusInfo> & _ContactStatusInfoObj,
                    const int & _SwingLinkInfoIndex,
                    const int & _ContactType):  FixedContactStatusInfo(_ContactStatusInfoObj),
                                                SwingLinkInfoIndex(_SwingLinkInfoIndex),
                                                ContactType(_ContactType){};
  std::vector<ContactStatusInfo> FixedContactStatusInfo;
  int SwingLinkInfoIndex;
  int ContactType;
};

struct InvertedPendulumInfo{
  InvertedPendulumInfo(){
    L = -1.0;
    g = -1.0;
    Theta = -1.0;
    Thetadot = -1.0;
    COMPos.setZero();
    COMVel.setZero();
    edge_a.setZero();
    edge_b.setZero();
  };
  InvertedPendulumInfo( const double & _L,
                        const double & _g,
                        const double & _Theta,
                        const double & _Thetadot,
                        const Vector3 & _COMPos,
                        const Vector3 & _COMVel): L(_L),
                                                  g(_g),
                                                  Theta(_Theta),
                                                  Thetadot(_Thetadot),
                                                  COMPos(_COMPos),
                                                  COMVel(_COMVel){};
  void setEdges(const Vector3 & _edge_a, const Vector3 & _edge_b){
    edge_a = _edge_a;
    edge_b = _edge_b;
  }
  double L, g;
  double Theta;
  double Thetadot;
  Vector3 COMPos;
  Vector3 COMVel;
  Vector3 edge_a, edge_b;
};

struct CubicSplineInfo{
  CubicSplineInfo(){
    ReadyFlag = false;
  };
  CubicSplineInfo(const std::vector<Vector3> & pVec_, const std::vector<double> & sVec_){
    ReadyFlag = false;
    CubicSplineInfoUpdate(pVec_, sVec_);
  }
  void CubicSplineInfoUpdate(const std::vector<Vector3> & pVec_, const std::vector<double> & sVec_){
    pVec = pVec_;
    sVec = sVec_;
    spline_x_y_z_init();
  }
  void spline_x_y_z_init(){
    const int n = pVec.size();
    std::vector<double> S = sVec;
    std::vector<double> X(n), Y(n), Z(n);
    for (int i = 0; i < n; i++){
      double s = sVec[i];
      X[i] = pVec[i].x;
      Y[i] = pVec[i].y;
      Z[i] = pVec[i].z;
    }
    tk::spline s_x, s_y, s_z;
    s_x.set_points(S,X);
    s_y.set_points(S,Y);
    s_z.set_points(S,Z);
    spline_x = s_x;
    spline_y = s_y;
    spline_z = s_z;
  }
  Vector3 s2Pos(double s) const{
    if(s<0.0) s = 0.0;
    if(s>1.0) s = 1.0;
    Vector3 Pos;
    Pos.x = spline_x(s);
    Pos.y = spline_y(s);
    Pos.z = spline_z(s);
    return Pos;
  }
  bool getReadyFlag() const { return ReadyFlag; }
  void setReadyFlag(const bool & ReadyFlag_) { ReadyFlag = ReadyFlag_; }
  tk::spline spline_x, spline_y, spline_z;
  bool ReadyFlag;
  std::vector<Vector3> pVec;
  std::vector<double> sVec;
};

struct StagePlanningInfo{
  StagePlanningInfo(){};

  bool ReadyFlag;
  double StageTime;
  std::vector<double> UpdatedConfig;
  std::vector<double> UpdatedVelocity;
};

#endif
