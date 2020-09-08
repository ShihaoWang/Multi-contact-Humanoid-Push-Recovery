// This function is used to calculate certain robot utility functions
#include "RobotInfo.h"
#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include <KrisLibrary/robotics/Inertia.h>
#include <random>
#include <limits>
#include <sys/stat.h>

extern std::vector<LinkInfo>   LinkInfoObj;
extern SDFInfo                 SDFInfoObj;
extern ReachabilityMap         ReachabilityMapObj;
extern AnyCollisionGeometry3D  TerrColGeomObj;

int FileIndexFinder(bool UpdateFlag, int WriteInt){
  string FileIndexName = "AtNo.txt";         // This file should be located in the "build" folder.
  ifstream FileIndexReader(FileIndexName);
  int FileIndex;
  string str_line;
  if (FileIndexReader.is_open()){
    while (getline (FileIndexReader,str_line))
    FileIndex = stoi(str_line);
    FileIndexReader.close();
  }
  else std:cerr<< "Unable to open FileIndex file";
  if(UpdateFlag){
    const char *FileIndexWriter_Name = FileIndexName.c_str();
    std::ofstream FileIndexWriter;
    FileIndexWriter.open(FileIndexWriter_Name);
    FileIndexWriter<<std::to_string(FileIndex + 1)<<"\n";
    FileIndexWriter.close();
  }
  if(WriteInt>0){
    const char *FileIndexWriter_Name = FileIndexName.c_str();
    std::ofstream FileIndexWriter;
    FileIndexWriter.open(FileIndexWriter_Name);
    FileIndexWriter<<std::to_string(WriteInt)<<"\n";
    FileIndexWriter.close(); 
  }
  return FileIndex;
}

static bool IsPathExist(const std::string &s){
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}

void PathFileManager(const string & CurrentCasePath){
  if(IsPathExist(CurrentCasePath))
    printf("%s exist!\n", CurrentCasePath.c_str());
  else {
    string str = "mkdir " + CurrentCasePath;
    const char *command = str.c_str();
    system(command);
  }
  // Let them be internal objects
  string str = "cd " + CurrentCasePath + " && ";

  str+="rm -f *Traj.txt && ";
  str+="rm -f *.path && ";
  str+="rm -f *InfoFile.txt && ";
  str+="rm -f PlanTime.txt && ";
  str+="rm -f *FailureMetric.txt && ";
  str+="rm -f *.bin && ";
  str+="rm -f *OptConfig*.config && ";
  str+="rm -f PlanRes.txt";
  const char *command = str.c_str();
  system(command);
}

void Vector3Appender(const char *File_Name, double Time_t, const Vector3 & Vector3Obj){
  std::ofstream CFWriter;
  CFWriter.open(File_Name, std::ios_base::app);
  CFWriter<<std::to_string(Time_t)<<" ";
  CFWriter<<std::to_string(Vector3Obj.x)<<" ";
  CFWriter<<std::to_string(Vector3Obj.y)<<" ";
  CFWriter<<std::to_string(Vector3Obj.z)<<"\n";
  CFWriter.close();  
}

void StateTrajAppender(const char *stateTrajFile_Name, const double & Time_t, const std::vector<double> & Configuration){
  std::ofstream StateTrajWriter;
  StateTrajWriter.open(stateTrajFile_Name, std::ios_base::app);
  StateTrajWriter<<std::to_string(Time_t)<<"\t";
  StateTrajWriter<<std::to_string(Configuration.size())<<"\t";
  for (int i = 0; i < Configuration.size()-1; i++)
    StateTrajWriter<<std::to_string(Configuration[i])<<" ";
  StateTrajWriter<<std::to_string(Configuration[Configuration.size()-1])<<"\n";
  StateTrajWriter.close();
}

void getCentroidalState(const Robot & SimRobot, Vector3 & COMPos, Vector3 & COMVel){
  // This function is used to get the centroidal position and velocity
  Vector3 COM = SimRobot.GetCOM();
  Matrix pCOMpq;
  SimRobot.GetCOMJacobian(pCOMpq);
  double COMVel_x =0.0, COMVel_y =0.0, COMVel_z =0.0;
  for (int i = 0; i < pCOMpq.n; i++){
    COMVel_x = COMVel_x + pCOMpq(0,i) * SimRobot.dq[i];
    COMVel_y = COMVel_y + pCOMpq(1,i) * SimRobot.dq[i];
    COMVel_z = COMVel_z + pCOMpq(2,i) * SimRobot.dq[i];
  }
  Vector3 COMVel_i(COMVel_x, COMVel_y, COMVel_z);
  COMPos = COM;
  COMVel = COMVel_i;
}

void PushInfoFileAppender(double SimTime, double Fx_t, double Fy_t, double Fz_t, const string & SpecificPath){
  std::ofstream PushInfoFileWriter;
  string PushInfoFileStr = SpecificPath + "PushInfoFile.txt";
  const char *PushInfoFileStr_Name = PushInfoFileStr.c_str();
  PushInfoFileWriter.open(PushInfoFileStr_Name, std::ios_base::app);
  PushInfoFileWriter<<std::to_string(SimTime)<<" "<< std::to_string(Fx_t)<<" "<<std::to_string(Fy_t)<<" "<<std::to_string(Fz_t)<<"\n";
  PushInfoFileWriter.close();
  return;
}

std::vector<Vector3> ActiveContactFinder(const Robot & SimRobot, const std::vector<ContactStatusInfo> & RobotContactInfo){
  // Note that this "active" is based on RobotContactInfo.
  std::vector<Vector3> ActContacts;
  for (int i = 0; i < LinkInfoObj.size(); i++){
    for (int j = 0; j < LinkInfoObj[i].LocalContacts.size(); j++){
      if(RobotContactInfo[i].LocalContactStatus[j]){
        Vector3 LinkiPjPos;
        SimRobot.GetWorldPosition(LinkInfoObj[i].LocalContacts[j], LinkInfoObj[i].LinkIndex, LinkiPjPos);
        ActContacts.push_back(LinkiPjPos);
      }
    }
  }
  return ActContacts;
}

double FailureMetricEval(const std::vector<PIPInfo> & PIPTotal){
  std::vector<double> pcp_pos;
  pcp_pos.reserve(PIPTotal.size());
  for (int i = 0; i < PIPTotal.size(); i++){
    double L = PIPTotal[i].L;
    double theta = PIPTotal[i].theta;

    double pcp_x = L * sin(theta);
    double pcp_xdot = PIPTotal[i].speed;
    double pcp_L = L * cos(theta);
    double pcp_g = PIPTotal[i].g;

    double pcp_pos_i;
    if(pcp_L>0.0){
      pcp_pos_i = pcp_x + pcp_xdot * sqrt(pcp_L/pcp_g);
      pcp_pos.push_back(pcp_pos_i);
    }
  }
  if(!pcp_pos.size()) return 0.0;
  return *min_element(pcp_pos.begin(), pcp_pos.end());;
}

void ContactPolytopeWriter(const std::vector<Vector3> & ActiveContact, const std::vector<PIPInfo> & PIPTotal, const SimPara & SimParaObj){
  std::vector<string> EdgeFileNames = SimParaObj.EdgeFileNames;
  std::ofstream fEdgeA;         fEdgeA.open(EdgeFileNames[0].c_str(), std::ios_base::app);
  std::ofstream fEdgeB;         fEdgeB.open(EdgeFileNames[1].c_str(), std::ios_base::app);
  std::ofstream fEdgeCOM;       fEdgeCOM.open(EdgeFileNames[2].c_str(), std::ios_base::app);
  std::ofstream fEdgex;         fEdgex.open(EdgeFileNames[3].c_str(), std::ios_base::app);
  std::ofstream fEdgey;         fEdgey.open(EdgeFileNames[4].c_str(), std::ios_base::app);
  std::ofstream fEdgez;         fEdgez.open(EdgeFileNames[5].c_str(), std::ios_base::app);
  std::ofstream fEdgeVetex;     fEdgeVetex.open(EdgeFileNames[6].c_str(), std::ios_base::app);

  for (auto PIPObj : PIPTotal){
    fEdgeA<<std::to_string(PIPObj.edge_a.x)<<" "<<std::to_string(PIPObj.edge_a.y)<<" "<<std::to_string(PIPObj.edge_a.z)<<" ";
    fEdgeB<<std::to_string(PIPObj.edge_b.x)<<" "<<std::to_string(PIPObj.edge_b.y)<<" "<<std::to_string(PIPObj.edge_b.z)<<" ";
    fEdgeCOM<<std::to_string(PIPObj.intersection.x)<<" "<<std::to_string(PIPObj.intersection.y)<<" "<<std::to_string(PIPObj.intersection.z)<<" ";
    fEdgex<<std::to_string(PIPObj.x_prime_unit.x)<<" "<<std::to_string(PIPObj.x_prime_unit.y)<<" "<<std::to_string(PIPObj.x_prime_unit.z)<<" ";
    fEdgey<<std::to_string(PIPObj.y_prime_unit.x)<<" "<<std::to_string(PIPObj.y_prime_unit.y)<<" "<<std::to_string(PIPObj.y_prime_unit.z)<<" ";
    fEdgez<<std::to_string(PIPObj.z_prime_unit.x)<<" "<<std::to_string(PIPObj.z_prime_unit.y)<<" "<<std::to_string(PIPObj.z_prime_unit.z)<<" ";
  }
  for (Vector3 ActiveContact_i:ActiveContact)
    fEdgeVetex<<std::to_string(ActiveContact_i.x)<<" "<<std::to_string(ActiveContact_i.y)<<" "<<std::to_string(ActiveContact_i.z)<<" ";
  fEdgeA<<"\n";                   fEdgeA.close();
  fEdgeB<<"\n";                   fEdgeB.close();
  fEdgeCOM<<"\n";                 fEdgeCOM.close();
  fEdgex<<"\n";                   fEdgex.close();
  fEdgey<<"\n";                   fEdgey.close();
  fEdgez<<"\n";                   fEdgez.close();
  fEdgeVetex<<"\n";               fEdgeVetex.close();
  return;
}

bool FailureChecker(const Robot & SimRobot){
  double DistTol = 0.01;
  std::vector<double> LinkTerrDistVec;
  for (int i = 5; i < SimRobot.q.size(); i++){
    if(!ReachabilityMapObj.EndEffectorIndexCheck.count(i)){
      if(SimRobot.geometry[i]->Distance(TerrColGeomObj)<DistTol)
        return true;
    }
  }
  return false;
}

bool AllPenetrationTester(const Robot & SimRobotObj){
  double DistTol = 1.0;
  for (int j = 0; j < LinkInfoObj.size(); j++){
    for (int i = 0; i < LinkInfoObj[j].LocalContacts.size(); i++) {
    Vector3 LocalPos;
    SimRobotObj.GetWorldPosition( LinkInfoObj[j].LocalContacts[i],
                                  LinkInfoObj[j].LinkIndex, LocalPos);
    double LocalPosDist = SDFInfoObj.SignedDistance(LocalPos);
    if(LocalPosDist<DistTol) DistTol = LocalPosDist;
    }
  }
  if(DistTol<0.0) return true;
  else return false;
}

bool PenetrationTester(const Robot & SimRobotObj, int SwingLinkInfoIndex){
  double DistTol = 1.0;
  for (int i = 0; i < LinkInfoObj[SwingLinkInfoIndex].LocalContacts.size(); i++) {
    Vector3 LocalPos;
    SimRobotObj.GetWorldPosition( LinkInfoObj[SwingLinkInfoIndex].LocalContacts[i],
                                  LinkInfoObj[SwingLinkInfoIndex].LinkIndex, LocalPos);
    double LocalPosDist = SDFInfoObj.SignedDistance(LocalPos);
    if(LocalPosDist<DistTol) DistTol = LocalPosDist;
  }
  if(DistTol<0.0) return true;
  else return false;
}

void StateLogger(WorldSimulation & Sim, LinearPath & CtrlStateTraj, LinearPath & PlanStateTraj, std::vector<double> & qDes, const SimPara & SimParaObj){
  const char *CtrlStateTrajStr_Name     = SimParaObj.CtrlStateTrajStr.c_str();
  const char *PlanStateTrajStr_Name     = SimParaObj.PlanStateTrajStr.c_str();
  CtrlStateTraj.Append(Sim.time,    Sim.world->robots[0]->q);
  StateTrajAppender(CtrlStateTrajStr_Name, Sim.time, Sim.world->robots[0]->q);
  if(qDes.size()==0) qDes = PlanStateTraj.milestones[PlanStateTraj.times.size()-1];
  StateTrajAppender(PlanStateTrajStr_Name, Sim.time, qDes);
  PlanStateTraj.Append(Sim.time,    Config(qDes));
  return;
}

bool OneHandAlreadyChecker(const ContactForm & ContactFormObj){
  if((ContactFormObj.FixedContactStatusInfo[2].LocalContactStatus[0])&&(ContactFormObj.SwingLinkInfoIndex == 3)) return true;
  if((ContactFormObj.FixedContactStatusInfo[3].LocalContactStatus[0])&&(ContactFormObj.SwingLinkInfoIndex == 2)) return true;
  return false;
}

void PlanTimeRecorder(double PlanTimeVal, const string & CurrentCasePath){
  // This function saves the total planning time needed for each planning procedure.
  string PlanTimeFileStr = CurrentCasePath + "PlanTime.txt";
  const char *PlanTimeFile_Name = PlanTimeFileStr.c_str();

  std::ofstream PlanTimeFile;
  PlanTimeFile.open(PlanTimeFile_Name, std::ios_base::app);
  PlanTimeFile<<std::to_string(PlanTimeVal);
  PlanTimeFile<<"\n";
  PlanTimeFile.close();
}

void PlanningInfoFileAppender(int PlanStageIndex, int TotalLinkNo, const string & CurrentCasePath, double CurTime){
  // This function saves the simulation time where contact planning happens, PlanStageIndex, and TotalLinkNo.
  std::ofstream PlanningInfoFileWriter;
  string PlanningInfoFileStr = CurrentCasePath + "PlanningInfoFile.txt";
  const char *PlanningInfoFileStr_Name = PlanningInfoFileStr.c_str();
  PlanningInfoFileWriter.open(PlanningInfoFileStr_Name, std::ios_base::app);
  PlanningInfoFileWriter<<std::to_string(PlanStageIndex)<<" "<< std::to_string(TotalLinkNo)<<" "<<std::to_string(CurTime)<<"\n";
  PlanningInfoFileWriter.close();
}

Vector3 getEndEffectorTranslationalVelocity(const Robot & SimRobot, int SwingLinkInfoIndex, const Config & JointVelocity){
  Vector3 EndEffectorVelocity; 
  SimRobot.GetWorldVelocity(LinkInfoObj[SwingLinkInfoIndex].AvgLocalContact,
                            LinkInfoObj[SwingLinkInfoIndex].LinkIndex, 
                            JointVelocity, EndEffectorVelocity);
  return EndEffectorVelocity;
}

Vector3 getEndEffectorAngularVelocity(const Robot & SimRobot, int SwingLinkInfoIndex, const Config & JointVelocity){
  Vector3 EndEffectorAngularVelocity; 
  SimRobot.GetWorldAngularVelocity( LinkInfoObj[SwingLinkInfoIndex].LinkIndex, 
                                    JointVelocity, EndEffectorAngularVelocity);
  return EndEffectorAngularVelocity;
}

Vector3 getPostionFromCubicCoeffs(double s, const std::vector<Vector3> & CubicCoeffs){
  Vector3 a = CubicCoeffs[0];
  Vector3 b = CubicCoeffs[1];
  Vector3 c = CubicCoeffs[2];
  Vector3 d = CubicCoeffs[3];
  Vector3 Position = a * s * s * s + b * s * s + c * s + d;
  return Position;
}

std::vector<double> LinearSpace(double a, double b, std::size_t N){
  double h = (b - a) / static_cast<double>(N-1);
  std::vector<double> xs(N);
  std::vector<double>::iterator x;
  double val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
      *x = val;
  }
  return xs;
}

void Vector3Writer(const std::vector<Vector3> & ContactPoints, const std::string & ContactPointFileName){
  if(!ContactPoints.size()) return;
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

QuaternionRotation getEndEffectorQuaternion(const Robot & SimRobotInner, int SwingLinkInfoIndex){
  // This function is used to get the Quaternion from end effector
  RobotLink3D EndEffectorLink = SimRobotInner.links[LinkInfoObj[SwingLinkInfoIndex].LinkIndex];

  Vector3 Local_x, Local_y, Local_z;
  Local_x.x = EndEffectorLink.T_World.R.data[0][0];
  Local_x.y = EndEffectorLink.T_World.R.data[0][1];
  Local_x.z = EndEffectorLink.T_World.R.data[0][2];

  Local_y.x = EndEffectorLink.T_World.R.data[1][0];
  Local_y.y = EndEffectorLink.T_World.R.data[1][1];
  Local_y.z = EndEffectorLink.T_World.R.data[1][2];

  Local_z.x = EndEffectorLink.T_World.R.data[2][0];
  Local_z.y = EndEffectorLink.T_World.R.data[2][1];
  Local_z.z = EndEffectorLink.T_World.R.data[2][2];

  Matrix3 RotMat(Local_x, Local_y, Local_z);
  QuaternionRotation EndEffectorQuaternion;
  EndEffectorQuaternion.setMatrix(RotMat);

  return EndEffectorQuaternion;
}

std::vector<int> SwingLinkChainSelector(const ReachabilityMap & ReachabilityMapObj, int SwingLinkInfoIndex, bool OneHandAlreadyFlag){
  std::vector<int> SwingLinkChain;
  if(SwingLinkInfoIndex<=1){
    SwingLinkChain = ReachabilityMapObj.EndEffectorChainIndices.at(SwingLinkInfoIndex);
  } else {
    if(OneHandAlreadyFlag){
      if(SwingLinkInfoIndex == 2){  
        SwingLinkChain = ReachabilityMapObj.Link34ToPivotal;
      } else SwingLinkChain = ReachabilityMapObj.Link27ToPivotal;
    } else SwingLinkChain = ReachabilityMapObj.EndEffectorChainIndices.at(SwingLinkInfoIndex);
  }
  return SwingLinkChain;
}


std::vector<Vector3> getEndEffectorXYZAxes(const Robot & SimRobotInner, int SwingLinkInfoIndex){
  RobotLink3D EndEffectorLink = SimRobotInner.links[LinkInfoObj[SwingLinkInfoIndex].LinkIndex];
  Vector3 EndEffectorInitxDir, EndEffectorInityDir, EndEffectorInitzDir;
  EndEffectorInitxDir.x = EndEffectorLink.T_World.R.data[0][0];
  EndEffectorInitxDir.y = EndEffectorLink.T_World.R.data[0][1];
  EndEffectorInitxDir.z = EndEffectorLink.T_World.R.data[0][2];

  EndEffectorInityDir.x = EndEffectorLink.T_World.R.data[1][0];
  EndEffectorInityDir.y = EndEffectorLink.T_World.R.data[1][1];
  EndEffectorInityDir.z = EndEffectorLink.T_World.R.data[1][2];

  EndEffectorInitzDir.x = EndEffectorLink.T_World.R.data[2][0];
  EndEffectorInitzDir.y = EndEffectorLink.T_World.R.data[2][1];
  EndEffectorInitzDir.z = EndEffectorLink.T_World.R.data[2][2];

  std::vector<Vector3> EndEffectorAxes = {EndEffectorInitxDir, EndEffectorInityDir, EndEffectorInitzDir}; 
  return EndEffectorAxes;
}


static vector<Vector3> Interpolation(const Vector3 & a, const Vector3 & b, int No){
  vector<Vector3> Pts(No);
  double ratio = (1.0)/(1.0 * No + 1.0);
  for (int i = 0; i < No; i++) {
    Pts[i] = a + (b-a) * ratio * (1.0 * i + 1.0);
  }
  return Pts;
}
std::vector<Vector3> BoxVertices(const Box3D & Box3DObj){
  // Here 4 more points will be sampled along each edge
  std::vector<Vector3> Vertices;
  Vector3 GlobalPoint;
  Vector3 LocalPoint(0.0, 0.0, 0.0);
  Box3DObj.fromLocal(LocalPoint, GlobalPoint);
  Vector3 xBox3DOffset = Box3DObj.dims.x * Box3DObj.xbasis;
  Vector3 yBox3DOffset = Box3DObj.dims.y * Box3DObj.ybasis;
  Vector3 zBox3DOffset = Box3DObj.dims.z * Box3DObj.zbasis;

  Vector3 A = GlobalPoint;
  Vector3 B = GlobalPoint + xBox3DOffset;
  Vector3 C = GlobalPoint + xBox3DOffset + zBox3DOffset;
  Vector3 D = GlobalPoint + zBox3DOffset;

  Vector3 E = GlobalPoint + yBox3DOffset;
  Vector3 F = GlobalPoint + xBox3DOffset + yBox3DOffset;
  Vector3 G = GlobalPoint + xBox3DOffset + zBox3DOffset + yBox3DOffset;
  Vector3 H = GlobalPoint + zBox3DOffset + yBox3DOffset;

  const int InterPtNo = 0;
  vector<Vector3> ABEdge = Interpolation(A, B, InterPtNo);
  vector<Vector3> BCEdge = Interpolation(B, C, InterPtNo);
  vector<Vector3> CDEdge = Interpolation(C, D, InterPtNo);
  vector<Vector3> DAEdge = Interpolation(D, A, InterPtNo);

  vector<Vector3> EFEdge = Interpolation(E, F, InterPtNo);
  vector<Vector3> FGEdge = Interpolation(F, G, InterPtNo);
  vector<Vector3> GHEdge = Interpolation(G, H, InterPtNo);
  vector<Vector3> HEEdge = Interpolation(H, E, InterPtNo);

  Vertices.push_back(A);
  Vertices.insert(Vertices.end(), ABEdge.begin(), ABEdge.end());

  Vertices.push_back(B);
  Vertices.insert(Vertices.end(), BCEdge.begin(), BCEdge.end());

  Vertices.push_back(C);
  Vertices.insert(Vertices.end(), CDEdge.begin(), CDEdge.end());

  Vertices.push_back(D);
  Vertices.insert(Vertices.end(), DAEdge.begin(), DAEdge.end());

  Vertices.push_back(E);
  Vertices.insert(Vertices.end(), EFEdge.begin(), EFEdge.end());

  Vertices.push_back(F);
  Vertices.insert(Vertices.end(), FGEdge.begin(), FGEdge.end());

  Vertices.push_back(G);
  Vertices.insert(Vertices.end(), GHEdge.begin(), GHEdge.end());

  Vertices.push_back(H);
  Vertices.insert(Vertices.end(), HEEdge.begin(), HEEdge.end());
  return Vertices;
}

void RobotConfigWriter(const std::vector<double> & Config, const string & user_path, const string &config_file_name){
  std::ofstream ConfigInfoFile;
  std::string config_file_path = user_path + config_file_name;
  ConfigInfoFile.open (config_file_path);
  ConfigInfoFile<<std::to_string(Config.size())<<"\t";
  for (int i = 0; i < Config.size(); i++)
    ConfigInfoFile << std::to_string(Config[i])<<" ";
  ConfigInfoFile.close();
}