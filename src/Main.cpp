#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>
#include "RobotInfo.h"

std::vector<LinkInfo>   NonlinearOptimizerInfo::RobotLinkInfo;
SignedDistanceFieldInfo NonlinearOptimizerInfo::SDFInfo;
AnyCollisionGeometry3D  NonlinearOptimizerInfo::TerrColGeom;

static void mainInner(string ExperimentFolderPath, int FileIndex, int ExpIndex, ReachabilityMap & RMObject, SelfLinkGeoInfo & SelfLinkGeoObj, SimPara & SimParaObj){
  
  RobotWorld world;
  SimGUIBackend Backend(&world);
  WorldSimulation& Sim = Backend.sim;
  
  string XMLFileStr =  ExperimentFolderPath + "Envi.xml";
  const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
  if(!Backend.LoadAndInitSim(XMLFile)){
    std::cerr<<XMLFileStr<< "file does not exist in that path!" << endl;
    return;
  }

  Robot SimRobot = *world.robots[0];
  string CurrentCasePath = ExperimentFolderPath + std::to_string(FileIndex) + "/";
  RobotConfigLoader(SimRobot, CurrentCasePath, "InitConfig.config");
  const std::string ContactStatusPath = CurrentCasePath + "ContactStatus.txt";
  std::vector<ContactStatusInfo> InitContactInfo = ContactStatusInfoLoader(ContactStatusPath);

  SimParaObj.CurrentCasePathUpdate(CurrentCasePath);
  FilePathManager(SimParaObj.CurrentCasePath);

  std::vector<double> InitConfig(SimRobot.q);
  std::vector<double> InitVelocity(SimRobot.q.size(), 0.0);
  std::vector<double> RobotConfigRef = InitVelocity;

  Sim.world->robots[0]->UpdateConfig(Config(InitConfig));
  Sim.world->robots[0]->dq = InitVelocity;
  Sim.controlSimulators[0].oderobot->SetConfig(Config(InitConfig));
  Sim.controlSimulators[0].oderobot->SetVelocities(Config(InitVelocity));
  
  // Enable terrain robot links contact
  const int NumberOfTerrains = Sim.world->terrains.size();
  for (int i = 0; i < NumberOfTerrains; i++){
    int terrainid = Sim.world->TerrainID(i);
    for (int j = 0; j < Sim.world->robots[0]->links.size(); j++){
      Sim.EnableContactFeedback(terrainid, world.RobotLinkID(0, j));    
    }
  }
  // int SimRes = SimulationTest(Sim, InitContactInfo, RMObject, SelfLinkGeoObj, SimParaObj);
  // PlanResWriter(CurrentCasePath, SimRes);
  return;
}

int main(){
   /* 1. Load the Contact Link file */
  const std::string UserFilePath = "../user/";
  const std::string ContactLinkPath = UserFilePath + "ContactLink.txt";
  int NumberOfContactPoints;
  NonlinearOptimizerInfo::RobotLinkInfo = ContactInfoLoader(ContactLinkPath, NumberOfContactPoints);
  const std::string TorsoLinkFilePath = UserFilePath + "TorsoLink.txt";
  std::vector<int> TorsoLink = TorsoLinkReader(TorsoLinkFilePath);
  const std::string SelfCollisionFreeLinkFilePath = UserFilePath + "SelfCollisionFreeLink.txt";
  std::vector<int> SelfCollisionFreeLink = TorsoLinkReader(SelfCollisionFreeLinkFilePath);

   /* 2. Setup Parameters for Algorithm */
  double PushDuration     = 0.25;
  double DetectionWait    = 0.25;

  // Three inner variables
  double TimeStep         = 0.025;
  double InitDuration     = 2.0;
  double TotalDuration    = 5.0;                     // Simulation lasts for 5s after initial duration

  double ForwardDuration  = 0.75;
  double PhaseRatio       = 0.6;
  double ReductionRatio   = 0.6;

  std::vector<std::string> ScenarioVec = { "flat_1Contact", "flat_2Contact", "uneven_1Contact", "uneven_2Contact"};  

  /* 3. Setup Parameters for Algorithm */
  for (auto Scenario: ScenarioVec){
    const std::string ExperimentFolderPath = "/home/motion/Desktop/Whole-Body-Planning-for-Push-Recovery-Data/" + Scenario + "/";
    RobotWorld worldObj;
    SimGUIBackend BackendObj(&worldObj);
    WorldSimulation& SimObj = BackendObj.sim;
    string XMLFileStrObj =  ExperimentFolderPath + "Envi.xml";
    const char* XMLFileObj = XMLFileStrObj.c_str();    // Here we must give abstract path to the file
    if(!BackendObj.LoadAndInitSim(XMLFileObj)){
      std::cerr<< XMLFileStrObj<<" file does not exist in that path!"<<endl;
      return -1;
    }

    const int GridsNo = 251;
    struct stat buffer;   // This is used to check whether "SDFSpecs.bin" exists or not.
    const string SDFPath = ExperimentFolderPath + "SDFs/";
    const string SDFSpecsName = SDFPath + "SDFSpecs.bin";
    if(stat (SDFSpecsName.c_str(), &buffer) == 0)
      NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldLoader(SDFPath, GridsNo);
    else
      NonlinearOptimizerInfo::SDFInfo = SignedDistanceFieldGene(SDFPath, worldObj, GridsNo);

    ReachabilityMap RMObject = ReachabilityMapGenerator(*worldObj.robots[0], NonlinearOptimizerInfo::RobotLinkInfo, TorsoLink);
    const int NumberOfTerrains = worldObj.terrains.size();
    std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*worldObj.terrains[0]);
    Meshing::TriMesh EnviTriMesh  = Terrain_ptr->geometry->AsTriangleMesh();

    for (int i = 0; i < NumberOfTerrains-1; i++){
      std::shared_ptr<Terrain> Terrain_ptr = std::make_shared<Terrain>(*worldObj.terrains[i+1]);
      Meshing::TriMesh EnviTriMesh_i  = Terrain_ptr->geometry->AsTriangleMesh();
      EnviTriMesh.MergeWith(EnviTriMesh_i);
    }
    AnyCollisionGeometry3D TerrColGeom(EnviTriMesh);
    NonlinearOptimizerInfo::TerrColGeom = TerrColGeom;

    const int ExpTotal = 100;
    int FileIndex = FileIndexFinder(false, -1);
    while (FileIndex<=ExpTotal){
      int ImpulseDirSize = 8;
      double ImpulseAngleUnit = M_PI/(1.0 * ImpulseDirSize);
      for (int i = 0; i < ImpulseDirSize; i++){
        double ImpulseAngle = 1.0 * i * ImpulseAngleUnit;
        double Impulse_x = cos(ImpulseAngle);
        double Impulse_y = sin(ImpulseAngle);
        Vector3 ImpulseDirection(Impulse_x, Impulse_y, 0.0);
        ImpulseDirection.setNormalized(ImpulseDirection);
        std::string ForceFilePath = ExperimentFolderPath + to_string(FileIndex) + "/Force.txt";
        ifstream ForceFile (ForceFilePath);
        string ForceStr;
        getline(ForceFile, ForceStr);
        double ImpulseForce = stod(ForceStr);
        ForceFile.close();
        SimPara SimParaObj( ImpulseForce, PushDuration,   DetectionWait, 
                            TimeStep,     InitDuration,   TotalDuration, 
                            ForwardDuration, PhaseRatio,  ReductionRatio);
        SimParaObj.setImpulseForceMax(ImpulseDirection);
        SelfLinkGeoInfo SelfLinkGeoObj(*worldObj.robots[0], RMObject.EndEffectorLink2Pivotal, SelfCollisionFreeLink);
        mainInner(ExperimentFolderPath, FileIndex, i+1, RMObject, SelfLinkGeoObj, SimParaObj);
      }
    FileIndex = FileIndexFinder(true, -1);
    FileIndex++; 
    }
    FileIndexFinder(true, 1);
  }
  return 1;
}