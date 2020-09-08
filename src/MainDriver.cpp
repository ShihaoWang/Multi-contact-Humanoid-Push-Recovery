#include "NonlinearOptimizerInfo.h"
#include "Control/PathController.h"
#include "Simulation/WorldSimulation.h"
#include <ode/ode.h>
#include "CommonHeader.h"

void MainDriver(const bool & SpecifiedFlag, const string & ExperimentFolderPath, int & FileIndex, const SelfCollisionInfo & SelfCollisionInfoObj){
  const int ExpTotal = 100;
  if(!SpecifiedFlag) FileIndex = FileIndexFinder(false, -1);
  while (FileIndex<=ExpTotal){
    int ImpulseDirSize = 8;
    double ImpulseAngleUnit = 2.0 * M_PI/(1.0 * ImpulseDirSize);
    for (int i = 0; i < ImpulseDirSize; i++){
      i = 4;
      double ImpulseAngle = 1.0 * i * ImpulseAngleUnit;
      Vector3 ImpulseDirection(cos(ImpulseAngle), sin(ImpulseAngle), 0.0);
      ImpulseDirection.setNormalized(ImpulseDirection);

      std::string ForceFilePath = ExperimentFolderPath + to_string(FileIndex) + "/Force.txt";
      ifstream ForceFile(ForceFilePath);
      string ForceStr;
      getline(ForceFile, ForceStr);
      double ImpulseForce = stod(ForceStr);
      ForceFile.close();

      std::vector<double> SimParaVec = getSimParaVec();
      SimPara SimParaObj(SimParaVec);
      SimParaObj.setImpulseForce(ImpulseForce, ImpulseDirection);
      SelfCollisionInfo SelfCollisionInfoObjInner = SelfCollisionInfoObj; 
      MainInner(ExperimentFolderPath, FileIndex, i+1, SelfCollisionInfoObjInner, SimParaObj);

    }
    if(!SpecifiedFlag)
      FileIndex = FileIndexFinder(true, -1);
    FileIndex++; 
  }
  if(!SpecifiedFlag) FileIndexFinder(true, 1);
  return;
}

