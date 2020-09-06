#include "CommonHeader.h"

void mainInner(string ExperimentFolderPath, int FileIndex, int ExpIndex, ReachabilityMap & ReachabilityMapObj, SelfLinkGeoInfo & SelfLinkGeoObj, SimPara & SimParaObj){
  
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
  RobotConfigLoader(SimRobot, CurrentFolderPath, "InitConfig.config");
  const std::string ContactStatusPath = CurrentFolderPath + "ContactStatus.txt";
  std::vector<ContactStatusInfo> InitContactInfo = ContactStatusInfoLoader(ContactStatusPath);

  string CurrentCasePath = CurrentFolderPath + to_string(ExpIndex) + "/";
  SimParaObj.CurrentCasePathUpdate(CurrentCasePath);
  FilePathManager(SimParaObj.CurrentCasePath);

  std::vector<double> InitConfig(SimRobot.q);
  std::vector<double> InitVelocity(SimRobot.q.size(), 0.0);

  Sim.world->robots[0]->UpdateConfig(Config(InitConfig));
  Sim.world->robots[0]->dq = InitVelocity;
  Sim.controlSimulators[0].oderobot->SetConfig(Config(InitConfig));
  Sim.controlSimulators[0].oderobot->SetVelocities(Config(InitVelocity));
  
  // Enable terrain robot links contact
  for (int i = 0; i < Sim.world->terrains.size(); i++){
    int terrainid = Sim.world->TerrainID(i);
    for (int j = 0; j < Sim.world->robots[0]->links.size(); j++){
      Sim.EnableContactFeedback(terrainid, world.RobotLinkID(0, j));    
    }
  }
  int SimRes = SimulationTest(Sim, InitContactInfo, ReachabilityMapObj, SelfLinkGeoObj, SimParaObj);

  RobotWorld failureworld;
  SimGUIBackend FailureBackend(&failureworld);
  WorldSimulation& FailureSim = FailureBackend.sim;
  if(!FailureBackend.LoadAndInitSim(XMLFile)){
    std::cerr<<XMLFileStr<< "file does not exist in that path!" << endl;
    return;
  }
  // Failure Trajectory Simulation 
  FailureSim.world->robots[0]->UpdateConfig(Config(InitConfig));
  FailureSim.world->robots[0]->dq = InitVelocity;
  FailureSim.controlSimulators[0].oderobot->SetConfig(Config(InitConfig));
  FailureSim.controlSimulators[0].oderobot->SetVelocities(Config(InitVelocity));
  int FailureRes = FailureTest(FailureSim, InitContactInfo, ReachabilityMapObj, SelfLinkGeoObj, SimParaObj);

  int ResVal = 0;
  if(SimRes){
    if(FailureRes) ResVal = 1;
    else ResVal = 0;
  } else ResVal = -1;
  PlanResWriter(CurrentCasePath, ResVal);
  return;
}