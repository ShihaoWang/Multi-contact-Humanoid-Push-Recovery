#include "CommonHeader.h"

static std::vector<ContactStatusInfo> InitContactInfo;
static int SimulationWithMethod(WorldSimulation & Sim, const std::vector<double> & InitConfig, SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj){
  std::vector<double> InitVelocity(InitConfig.size(), 0.0);
  Sim.world->robots[0]->UpdateConfig(Config(InitConfig));
  Sim.world->robots[0]->dq = InitVelocity;
  Sim.controlSimulators[0].oderobot->SetConfig(Config(InitConfig));
  Sim.controlSimulators[0].oderobot->SetVelocities(Config(InitVelocity));
  return SimulationWithMethod(Sim, InitContactInfo, SelfCollisionInfoObj, SimParaObj);
}

static int SimulationWithoutMethod(string XMLFileStr, const std::vector<double> & InitConfig, SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj){

  const char* XMLFile = XMLFileStr.c_str();    // Here we must give abstract path to the file
  RobotWorld failureworld;
  SimGUIBackend FailureBackend(&failureworld);
  WorldSimulation& FailureSim = FailureBackend.sim;
  if(!FailureBackend.LoadAndInitSim(XMLFile)){
    std::cerr<<XMLFileStr<< "file does not exist in that path!" << endl;
    return -1;
  }
  // Failure Trajectory Simulation 
  std::vector<double> InitVelocity(InitConfig.size(), 0.0);
  FailureSim.world->robots[0]->UpdateConfig(Config(InitConfig));
  FailureSim.world->robots[0]->dq = InitVelocity;
  FailureSim.controlSimulators[0].oderobot->SetConfig(Config(InitConfig));
  FailureSim.controlSimulators[0].oderobot->SetVelocities(Config(InitVelocity));
  // int FailureRes = FailureTest(FailureSim, InitContactInfo, ReachabilityMapObj, SelfLinkGeoObj, SimParaObj);
  // return FailureRes;
  return 1;
}

void MainInner(string ExperimentFolderPath, int FileIndex, int ExpIndex, SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj){
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
  string CurrentFolderPath = ExperimentFolderPath + std::to_string(FileIndex) + "/";
  std::vector<double> InitConfig = RobotConfigLoader(CurrentFolderPath, "InitConfig.config");
  if(!InitConfig.size()){
    std::cerr<<"Empty InitConfig.config at "<< CurrentFolderPath <<std::endl;
    return; 
  }
  const std::string ContactStatusPath = CurrentFolderPath + "ContactStatus.txt";
  InitContactInfo = ContactStatusInfoLoader(ContactStatusPath);

  string CurrentCasePath = CurrentFolderPath + to_string(ExpIndex) + "/";
  SimParaObj.CurrentCasePathUpdate(CurrentCasePath);
  PathFileManager(SimParaObj.CurrentCasePath);

  int SimRes      = SimulationWithMethod(   Sim, InitConfig, SelfCollisionInfoObj, SimParaObj);
  int FailureRes  = SimulationWithoutMethod(XMLFileStr, InitConfig, SelfCollisionInfoObj, SimParaObj);

  int ResVal = 0;
  if(SimRes){
    if(FailureRes) ResVal = 1;
    else ResVal = 0;
  } else ResVal = -1;
  // PlanResWriter(CurrentCasePath, ResVal);
  return;
}