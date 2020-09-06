// This function is used to load in the robot's specification
#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"

static void SDFWriter(const string & SDFPath, const std::vector<double> & SDFVector, const std::vector<double> & SDFSpecs)
{
  // This function will write the computed SDFTensor and SDFSpecs into file
  FILE * SDFTensorFile = NULL;
  const std::string SDFTensorFileNameStr = SDFPath + "SDFTensor.bin";
  const char* SDFTensorFileName = SDFTensorFileNameStr.c_str() ;
  SDFTensorFile = fopen(SDFTensorFileName, "wb");
  fwrite(&SDFVector[0], sizeof(double), SDFVector.size(), SDFTensorFile);
  fclose(SDFTensorFile);

  FILE * SDFSpecsFile = NULL;
  const std::string SDFSpecsFileNameStr = SDFPath + "SDFSpecs.bin";
  const char* SDFSpecsFileName = SDFSpecsFileNameStr.c_str();
  SDFSpecsFile = fopen(SDFSpecsFileName, "wb");
  fwrite(&SDFSpecs[0], sizeof(double), SDFSpecs.size(), SDFSpecsFile);
  fclose(SDFSpecsFile);
}


std::vector<LinkInfo> LinkInfoLoader(string ContactLinkFile){
  string str_line, str_keyword;
  str_keyword = "Link";
  ContactLinkFile+="LinkInfo.txt";
  ifstream linkinfofile (ContactLinkFile);
  std::vector<LinkInfo> LinkInfoVec;
  LinkInfo contact_link_i;
  int LinkIndex = -1;
  if (linkinfofile.is_open()){
    while (getline (linkinfofile,str_line) ){
      if (str_line.find(str_keyword) != string::npos){
        str_line.erase(str_line.begin(), str_line.begin()+4);
        int link_index = stoi(str_line);
        LinkInfo contact_link_point_i(link_index);
        contact_link_i = contact_link_point_i;
        LinkInfoVec.push_back(contact_link_i);
        LinkIndex = LinkIndex + 1;
      } else {
        // Each row will be converted into a Vector3 and push into LinkInfo
        istringstream ss(str_line);     // Here the string row can be separated by the comma
        string link_coordinate_i;
        vector <double> link_coordinates;
        while (ss >> link_coordinate_i){
          link_coordinate_i.erase(link_coordinate_i.end()-1, link_coordinate_i.end());
          link_coordinates.push_back(stod(link_coordinate_i));
        }
        Vector3 link_coordinates_Vect3(link_coordinates[0], link_coordinates[1], link_coordinates[2]);
        LinkInfoVec[LinkIndex].AddLocalConact(link_coordinates_Vect3);
      }
    }
    for (int i = 0; i < LinkInfoVec.size(); i++)
      LinkInfoVec[i].AvgContactUpdate();
    linkinfofile.close();
  }
  else std::cerr << "Unable to open file";

  if (!LinkInfoVec.size())
    std::cerr<<"Robot Contact Link Info failed to be loaded!"<<"\n";
  
  return LinkInfoVec;
}

std::vector<ContactStatusInfo> ContactStatusInfoLoader(const string & ContactStatusFile){
  string str_line, str_keyword;
  str_keyword = "Link";

  ifstream contactstatusinfofile (ContactStatusFile);
  std::vector<ContactStatusInfo> ContactStatusInfoVec;
  ContactStatusInfo contact_link_i;
  int LinkIndex = -1;
  if (contactstatusinfofile.is_open()){
    while (getline (contactstatusinfofile,str_line)){
      if (str_line.find(str_keyword) != string::npos){
        str_line.erase (str_line.begin(), str_line.begin()+4);
        int link_index = stoi(str_line);
        ContactStatusInfo contact_link_point_i(link_index);
        contact_link_i = contact_link_point_i;
        ContactStatusInfoVec.push_back(contact_link_i);
        LinkIndex = LinkIndex + 1;
      } else ContactStatusInfoVec[LinkIndex].AddLocalConactStatus(std::stoi(str_line));
    }
    contactstatusinfofile.close();
  }
  else std::cerr << "\nUnable to open file";

  if (!ContactStatusInfoVec.size())
    std::cerr<<"\nRobot Contact Status Info failed to be loaded!"<<"\n";
  
  return ContactStatusInfoVec;
}

std::vector<int> LinkIndicesLoader(const string & UserFilePath, const string & FileName){
  string TorsoLinkFilePath = UserFilePath + FileName;
  ifstream TorsoLinkFile (TorsoLinkFilePath);
  std::vector<int> TorsoLinkVec;
  int LinkIndex = -1;
  if (TorsoLinkFile.is_open()){
    string str_line;
    while (getline (TorsoLinkFile, str_line) ){
      int link_index = stoi(str_line);
      TorsoLinkVec.push_back(link_index);
    }
    TorsoLinkFile.close();
  }
  else std::cerr << "Unable to open file " <<TorsoLinkFilePath<<" does not exist!\n";
  if (!TorsoLinkVec.size()) 
    std::cerr<<TorsoLinkFilePath<<" failed to be loaded!"<<"\n";
  return TorsoLinkVec;
}

SDFInfo SDFInfoGene(const string & SDFPath, const RobotWorld& WorldObj, const int& GridsNo){
  double resolution = 0.025;
  const int NumberOfTerrains = WorldObj.terrains.size();

  std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*WorldObj.terrains[0]);
  Meshing::TriMesh EnviTriMesh  = Terrain_ptr->geometry->AsTriangleMesh();

  // This step is used to merge the meshes into a single one.
  for (int i = 0; i < NumberOfTerrains-1; i++){
    std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*WorldObj.terrains[i+1]);
    Meshing::TriMesh EnviTriMesh_i  = Terrain_ptr->geometry->AsTriangleMesh();
    EnviTriMesh.MergeWith(EnviTriMesh_i);
  }
  Meshing::VolumeGrid SDFGrid;
  CollisionMesh EnviTriMeshTopology(EnviTriMesh);
  EnviTriMeshTopology.InitCollisions();
  EnviTriMeshTopology.CalcTriNeighbors();
  MeshToImplicitSurface_FMM(EnviTriMeshTopology, SDFGrid, resolution);
  // cout<<"FMM grid bounding box "<<SDFGrid.bb<<endl;
  // Now it is time to calculate SignedDistanceFieldInfo struct obj from SDFGrid

  // The estimated sizes of the environment
  double BB_x_min = SDFGrid.bb.bmin[0];
  double BB_x_max = SDFGrid.bb.bmax[0];

  double BB_y_min = SDFGrid.bb.bmin[1];
  double BB_y_max = SDFGrid.bb.bmax[1];

  double BB_z_min = SDFGrid.bb.bmin[2];
  double BB_z_max = SDFGrid.bb.bmax[2];

  double BB_x_length = BB_x_max - BB_x_min;
  double BB_y_length = BB_y_max - BB_y_min;
  double BB_z_length = BB_z_max - BB_z_min;

  double ExtCoeff = 0.0;

  // The estimated sizes of the environment
  double Envi_x_min = BB_x_min - ExtCoeff * BB_x_length;
  double Envi_x_max = BB_x_max + ExtCoeff * BB_x_length;

  double Envi_y_min = BB_y_min - ExtCoeff * BB_y_length;
  double Envi_y_max = BB_y_max + ExtCoeff * BB_y_length;

  double Envi_z_min = BB_z_min - ExtCoeff * BB_z_length;
  double Envi_z_max = BB_z_max + ExtCoeff * BB_z_length;

  double Envi_x_length = Envi_x_max - Envi_x_min;
  double Envi_y_length = Envi_y_max - Envi_y_min;
  double Envi_z_length = Envi_z_max - Envi_z_min;

  double Envi_x_unit = Envi_x_length/(1.0* GridsNo - 1.0);
  double Envi_y_unit = Envi_y_length/(1.0* GridsNo - 1.0);
  double Envi_z_unit = Envi_z_length/(1.0* GridsNo - 1.0);

  std::vector<double> Envi_x_coor(GridsNo), Envi_y_coor(GridsNo), Envi_z_coor(GridsNo);
  for (int i = 0; i < GridsNo; i++){
    Envi_x_coor[i] = Envi_x_min + (1.0 * i) * Envi_x_unit;
    Envi_y_coor[i] = Envi_y_min + (1.0 * i) * Envi_y_unit;
    Envi_z_coor[i] = Envi_z_min + (1.0 * i) * Envi_z_unit;
  }

  // Generation of the SDFTensor structure
  Eigen::Tensor<double,3> SDFTensor(GridsNo, GridsNo, GridsNo);
  SDFTensor.setZero();

  Vector3 GridPoint;
  double GridPointDist, GridPoint_x, GridPoint_y, GridPoint_z;
  std::vector<double> SDFVector;
  SDFVector.reserve(GridsNo * GridsNo * GridsNo);
  for (int i = 0; i < GridsNo; i++){
    GridPoint_x = Envi_x_coor[i];
    for (int j = 0; j < GridsNo; j++){
      GridPoint_y = Envi_y_coor[j];
      for (int k = 0; k < GridsNo; k++){
        GridPoint_z = Envi_z_coor[k];
        GridPoint.set(GridPoint_x, GridPoint_y, GridPoint_z);
        GridPointDist = SDFGrid.TrilinearInterpolate(GridPoint);
        SDFTensor(i,j,k) = GridPointDist;
        SDFVector.push_back(GridPointDist);
      }
    }
  }

  std::vector<double> SDFSpecs;
  SDFSpecs.push_back(Envi_x_min);             SDFSpecs.push_back(Envi_x_max);
  SDFSpecs.push_back(Envi_y_min);             SDFSpecs.push_back(Envi_y_max);
  SDFSpecs.push_back(Envi_z_min);             SDFSpecs.push_back(Envi_z_max);

  SDFSpecs.push_back(Envi_x_unit);            SDFSpecs.push_back(Envi_y_unit);            SDFSpecs.push_back(Envi_z_unit);
  SDFSpecs.push_back(Envi_x_length);          SDFSpecs.push_back(Envi_y_length);          SDFSpecs.push_back(Envi_z_length);
  SDFSpecs.push_back(GridsNo);
  SDFWriter(SDFPath, SDFVector, SDFSpecs);
  SDFInfo SDFInfoObj(SDFTensor, SDFSpecs);
  return SDFInfoObj;
}


SDFInfo SDFInfoLoader(const string & SDFPath, const int GridsNo){
  std::cout<<"Loading "<<SDFPath<<endl;
  // This function will read in the computed SDF_File into a Eigen::Tensor
  const std::string SDF_FileNameStr = SDFPath + "SDFTensor.bin";
  const char* SDF_FileName = SDF_FileNameStr.c_str();
  FILE* SDF_File = fopen(SDF_FileName, "rb");
  std::vector<double> SDFVector(GridsNo * GridsNo * GridsNo);
  fread(&SDFVector[0], sizeof(double), GridsNo * GridsNo * GridsNo, SDF_File);
  fclose(SDF_File);

  // The next job is to write to the Eigen::Tensor
  Eigen::Tensor<double,3> SDFTensor(GridsNo,GridsNo, GridsNo);
  int SDFIndex = 0;
  for (int i = 0; i < GridsNo; i++){
    for (int j = 0; j < GridsNo; j++){
      for (int k = 0; k < GridsNo; k++){
        SDFTensor(i,j,k) = SDFVector[SDFIndex];
        SDFIndex +=1;
      }
    }
  }
  FILE * SDFSpecsFile = NULL;
  std::vector<double> SDFSpecs(13);
  const std::string SDFSpecsFileNameStr = SDFPath + "SDFSpecs.bin";
  const char* SDFSpecsFileName = SDFSpecsFileNameStr.c_str();
  SDFSpecsFile = fopen(SDFSpecsFileName, "rb");
  fread(&SDFSpecs[0], sizeof(double), 13, SDFSpecsFile);
  fclose(SDFSpecsFile);
  SDFInfo SDFInfoObj(SDFTensor, SDFSpecs);
  return SDFInfoObj;
}


static std::vector<PolarPoint> PolarPointLayerGene(double r, int n){
  std::vector<PolarPoint> PolarPointLayer;
  PolarPointLayer.reserve(n);
  double phi = M_PI * (3.0 - sqrt(5.0));      // golden angle in radians
  const double gr=(sqrt(5.0) + 1.0) / 2.0;    // golden ratio = 1.6180339887498948482
  const double ga=(2.0 - gr) * (2.0*M_PI);    // golden angle = 2.39996322972865332
  for (int i = 1; i <= n; i++) {
    const double lat = asin(-1.0 + 2.0 * double(i) / (n+1));
    const double lon = ga * i;

    const double x = cos(lon)*cos(lat);
    const double y = sin(lon)*cos(lat);
    const double z = sin(lat);

    double PointNorm = std::sqrt(x * x + y * y + z * z);
    double Ratio = PointNorm/r;
    Vector3 PolarPointPos(x/Ratio, y/Ratio, z/Ratio);
    PolarPoint PolarPoint_i(r, PolarPointPos);
    PolarPointLayer.push_back(PolarPoint_i);
  }
  return PolarPointLayer;
}

// Here this function is used to generate a sufficiently dense reachability map for end effector(s)
ReachabilityMap ReachabilityMapGenerator(const Robot & SimRobotObj, const std::vector<LinkInfo> & LinkInfoObj, const std::vector<int> & TorsoLinkIndices){
  Robot SimRobot = SimRobotObj;
  double  RadiusMax = 0.95;
  int     LayerSize = 61;
  double  RadiusUnit = RadiusMax/(LayerSize * 1.0);
  double  RadiusMin = RadiusUnit;
  
  ReachabilityMap ReachabilityMapObj(RadiusMin, RadiusMax, LayerSize);

  std::map<int, std::vector<PolarPoint>> PolarPointLayers;
  int PointSize = 0;
  for (int i = 0; i < LayerSize; i++){
    double Radius = (1.0 * i + 1.0) * RadiusUnit;
    int LayerPointLimit = (i + 1) * (i + 1);
    std::vector<PolarPoint> PolarPointLayer = PolarPointLayerGene(Radius, LayerPointLimit);
    PointSize+=PolarPointLayer.size();
    PolarPointLayers[i] = PolarPointLayer;
  }
  ReachabilityMapObj.PolarPointLayers = PolarPointLayers;

  // The next step is to estimate the end effector radius.
  int DOF = SimRobot.q.size();
  std::vector<double> DefaultConfig(DOF);
  for (int i = 0; i < DOF; i++)
    DefaultConfig[i] = 0.0;
  
  // These two values are modified to ensure the arms are straight.
  DefaultConfig[23] = SimRobot.qMin[23];
  DefaultConfig[30] = SimRobot.qMax[30];
  SimRobot.UpdateConfig(Config(DefaultConfig));
  SimRobot.UpdateGeometry();

  const int RobotLinkInfoSize = LinkInfoObj.size();

  std::vector<double> EndEffectorChainRadius(RobotLinkInfoSize);
  std::vector<double> EndEffectorGeometryRadius(RobotLinkInfoSize);
  std::vector<int>    EndEffectorLinkIndex(RobotLinkInfoSize);
  std::vector<int>    EndEffectorPivotalIndex(RobotLinkInfoSize);

  std::map<int, std::vector<int>> EndEffectorChainIndices;
  std::map<int, int> EndEffectorIndexCheck;

  for (int i = 0; i < RobotLinkInfoSize; i++){
    int ParentIndex = -1;
    int CurrentIndex = LinkInfoObj[i].LinkIndex;
    EndEffectorIndexCheck[CurrentIndex] = 1;
    EndEffectorLinkIndex[i] = LinkInfoObj[i].LinkIndex;
    std::vector<int> EndEffectorLink2PivotalIndex;
    EndEffectorLink2PivotalIndex.push_back(CurrentIndex);
    while(std::find(TorsoLinkIndices.begin(), TorsoLinkIndices.end(), ParentIndex)==TorsoLinkIndices.end()) {
      ParentIndex = SimRobot.parents[CurrentIndex];
      CurrentIndex = ParentIndex;
      EndEffectorLink2PivotalIndex.push_back(CurrentIndex);
    }
    EndEffectorLink2PivotalIndex.pop_back();
    Vector3 PivotalPos, EndPos, PivotalRef(0.0, 0.0, 0.0);
    SimRobot.GetWorldPosition(PivotalRef, EndEffectorLink2PivotalIndex[EndEffectorLink2PivotalIndex.size()-1], PivotalPos);
    SimRobot.GetWorldPosition(LinkInfoObj[i].AvgLocalContact, LinkInfoObj[i].LinkIndex, EndPos);
    EndEffectorPivotalIndex[i] = EndEffectorLink2PivotalIndex[EndEffectorLink2PivotalIndex.size()-1];
    EndEffectorChainIndices[i] = EndEffectorLink2PivotalIndex;

    std::vector<double> CollisionRadius(LinkInfoObj[i].LocalContacts.size());
    for (int j = 0; j < LinkInfoObj[i].LocalContacts.size(); j++){
      Vector3 Center2Edge = LinkInfoObj[i].LocalContacts[j] - LinkInfoObj[i].AvgLocalContact;
      CollisionRadius[j] = Center2Edge.norm();
    }
    EndEffectorGeometryRadius[i] = *std::max_element(CollisionRadius.begin(), CollisionRadius.end());
    Vector3 Pivotal2End = PivotalPos - EndPos;
    double Pivotal2EndRadius = Pivotal2End.norm();
    EndEffectorChainRadius[i] = Pivotal2EndRadius;
  }
  
  ReachabilityMapObj.EndEffectorChainRadius = EndEffectorChainRadius;
  ReachabilityMapObj.EndEffectorGeometryRadius = EndEffectorGeometryRadius;
  ReachabilityMapObj.EndEffectorLinkIndex = EndEffectorLinkIndex;
  ReachabilityMapObj.EndEffectorPivotalIndex = EndEffectorPivotalIndex;

  ReachabilityMapObj.EndEffectorChainIndices = EndEffectorChainIndices;
  ReachabilityMapObj.EndEffectorIndexCheck = EndEffectorIndexCheck;

  std::vector<int> Link34ToPivotal{34, 33, 32, 31, 30, 29};
  std::vector<int> Link27ToPivotal{27, 26, 25, 24 ,23 ,22};
  ReachabilityMapObj.Link34ToPivotal = Link34ToPivotal;
  ReachabilityMapObj.Link27ToPivotal = Link27ToPivotal;

  ReachabilityMapObj.PointSize = PointSize;
  return ReachabilityMapObj;
}
