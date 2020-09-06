#include "NonlinearOptimizerInfo.h"
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>
#include "CommonHeader.h"

// Three Global Variables 
std::vector<LinkInfo>   LinkInfoObj;
SDFInfo                 SDFInfoObj;
AnyCollisionGeometry3D  TerrColGeomObj;

int main(int argc, char** argv){

   /* 0. Load the Contact Link file */
  const std::string UserFilePath = "../user/";
  LinkInfoObj = LinkInfoLoader(UserFilePath);
  std::vector<int> TorsoLinkIndices             = LinkIndicesLoader(UserFilePath, "TorsoLink.txt");
  std::vector<int> SelfCollisionFreeLinkIndices = LinkIndicesLoader(UserFilePath, "SelfCollisionFreeLink.txt");

   /* 1. Setup Parameters for Algorithm */
  double PushDuration     = 0.50;
  double DetectionWait    = 0.25;

  double TimeStep         = 0.025;
  double InitDuration     = 1.0;
  double TotalDuration    = 5.0;                     // Simulation lasts for 5s after initial duration

  double ForwardDurationSeed  = 0.5;
  double PhaseRatio           = 0.6;
  double ReductionRatio       = 0.5;

  std::vector<std::string> ScenarioVec;
  bool SpecifiedFlag = false;
  int FileIndex = 0;
  if(argc>1){
    if(argc<3){
      std:cerr<<"Two Inputs Are Needed But Only One Input Is Given!"<<endl;
      return -1;
    }
    string ScenarioName(argv[1]);
    ScenarioVec.push_back(ScenarioName);
    FileIndex = atoi(argv[2]);
    SpecifiedFlag = true;
  }else{
    std::printf("Using Regular Test Flow!\n");
    std::vector<std::string> ScenarioVecOption = { "flat_1Contact", "flat_2Contact", "uneven_1Contact", "uneven_2Contact"};  
    ScenarioVec = ScenarioVecOption;
  }

  for (auto Scenario: ScenarioVec){
    std::string FolderName = "Whole-Body-Planning-for-Push-Recovery-Data";
    std::string ExperimentFolderPath = "/home/motion/Desktop/" + FolderName + "/" + Scenario + "/";
    RobotWorld worldObj;
    SimGUIBackend BackendObj(&worldObj);
    WorldSimulation& SimObj = BackendObj.sim;
    string XMLFileStrObj =  ExperimentFolderPath + "Envi.xml";
    const char* XMLFileObj = XMLFileStrObj.c_str();    // Here we must give abstract path to the file
    if(!BackendObj.LoadAndInitSim(XMLFileObj)){
      std::cerr<< XMLFileStrObj<<" file does not exist in that path!"<<endl;
      return -1;
    }
  
    /* 2. SDF */
    const int GridsNo = 251;
    struct stat buffer;   // This is used to check whether "SDFSpecs.bin" exists or not.
    const string SDFPath = ExperimentFolderPath + "SDFs/";
    const string SDFSpecsName = SDFPath + "SDFSpecs.bin";
    if(stat (SDFSpecsName.c_str(), &buffer) == 0)
      SDFInfoObj = SDFInfoLoader(SDFPath, GridsNo);
    else
      SDFInfoObj = SDFInfoGene(SDFPath, worldObj, GridsNo);
  
    /* 3. SDF, ReachabilityMap and TerrColGeomObj*/
    ReachabilityMap ReachabilityMapObj = ReachabilityMapGenerator(*worldObj.robots[0], LinkInfoObj, TorsoLinkIndices);
    const int NumberOfTerrains = worldObj.terrains.size();
    std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*worldObj.terrains[0]);
    Meshing::TriMesh EnviTriMesh  = Terrain_ptr->geometry->AsTriangleMesh();  
    for (int i = 0; i < NumberOfTerrains-1; i++){
      std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*worldObj.terrains[i+1]);
      Meshing::TriMesh EnviTriMesh_i  = Terrain_ptr->geometry->AsTriangleMesh();
      EnviTriMesh.MergeWith(EnviTriMesh_i);
    }
    AnyCollisionGeometry3D TerrColGeom(EnviTriMesh);
    TerrColGeomObj = TerrColGeom;

    // const int ExpTotal = 100;
    // if(!SpecifiedFlag) FileIndex = FileIndexFinder(false, -1);
    // while (FileIndex<=ExpTotal){
    //   int ImpulseDirSize = 8;
    //   double ImpulseAngleUnit = 2.0 * M_PI/(1.0 * ImpulseDirSize);
    //   for (int i = 0; i < ImpulseDirSize; i++){
    //     double ImpulseAngle = 1.0 * i * ImpulseAngleUnit;
    //     double Impulse_x = cos(ImpulseAngle);
    //     double Impulse_y = sin(ImpulseAngle);
    //     Vector3 ImpulseDirection(Impulse_x, Impulse_y, 0.0);
    //     ImpulseDirection.setNormalized(ImpulseDirection);
    //     std::string ForceFilePath = ExperimentFolderPath + to_string(FileIndex) + "/Force.txt";
    //     ifstream ForceFile (ForceFilePath);
    //     string ForceStr;
    //     getline(ForceFile, ForceStr);
    //     double ImpulseForce = stod(ForceStr);
    //     ForceFile.close();
    //     SimPara SimParaObj( ImpulseForce, PushDuration,   DetectionWait, 
    //                         TimeStep,     InitDuration,   TotalDuration, 
    //                         FootForwardDuration, HandForwardDuration, PhaseRatio,  ReductionRatio);
    //     SimParaObj.setImpulseForceMax(ImpulseDirection);
    //     SelfLinkGeoInfo SelfLinkGeoObj(*worldObj.robots[0], ReachabilityMapObj.EndEffectorLink2Pivotal, SelfCollisionFreeLink);
    //     mainInner(ExperimentFolderPath, FileIndex, i+1, ReachabilityMapObj, SelfLinkGeoObj, SimParaObj);
    //   }
    //   if(!SpecifiedFlag)
    //     FileIndex = FileIndexFinder(true, -1);
    //   FileIndex++; 
    // }
    // if(!SpecifiedFlag) FileIndexFinder(true, 1);
  }
  return 1;
}