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
