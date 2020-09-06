#include "NonlinearOptimizerInfo.h"
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>
#include "CommonHeader.h"

// Three Global Variables 
std::vector<LinkInfo>   LinkInfoObj;
SDFInfo                 SDFInfoObj;
ReachabilityMap         ReachabilityMapObj;
AnyCollisionGeometry3D  TerrColGeomObj;

int main(int argc, char** argv){

   /* 0. Load the End Effector Link file */
  const std::string UserFilePath = "../user/";
  LinkInfoObj = LinkInfoLoader(UserFilePath);

  std::vector<int> TorsoLinkIndices             = LinkIndicesLoader(UserFilePath, "TorsoLink.txt");
  std::vector<int> SelfCollisionFreeLinkIndices = LinkIndicesLoader(UserFilePath, "SelfCollisionFreeLink.txt");

  std::vector<std::string> ScenarioVec;
  bool SpecifiedFlag = false;
  int FileIndex = 1;
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
    /* 2. SDFInfoObj */
    SDFInfoObj = SDFInfoObjInit(ExperimentFolderPath, worldObj);
    /* 3. ReachabilityMapObj */
    ReachabilityMapObj = ReachabilityMapInit(*worldObj.robots[0], LinkInfoObj, TorsoLinkIndices);
    /* 4. TerrColGeomObj */
    TerrColGeomObj = TerrColGeomObjInit(worldObj);
    /* 5. SelfCollisionInfoObj */
    SelfCollisionInfo SelfCollisionInfoObj(*worldObj.robots[0], ReachabilityMapObj.EndEffectorChainIndices, SelfCollisionFreeLinkIndices);

    MainDriver(SpecifiedFlag, ExperimentFolderPath, FileIndex, SelfCollisionInfoObj);
  }
  return 1;
}