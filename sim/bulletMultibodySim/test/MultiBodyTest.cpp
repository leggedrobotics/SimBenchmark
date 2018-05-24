//
// Created by kangd on 24.05.18.
//


#include "api/b3RobotSimulatorClientAPI_NoGUI.h"

int main(int argc, char* argv[])
{
  b3RobotSimulatorClientAPI_NoGUI* sim = new b3RobotSimulatorClientAPI_NoGUI();

  bool isConnected = sim->connect(eCONNECT_SHARED_MEMORY);
  if (!isConnected)
  {
    printf("Using Direct mode\n");
    isConnected = sim->connect(eCONNECT_DIRECT);
  } else
  {
    printf("Using shared memory\n");
  }
  //remove all existing objects (if any)
  sim->resetSimulation();

//  std::string groundpath(__FILE__);
//  while (groundpath.back() != '/')
//    groundpath.erase(groundpath.size() - 1, 1);
//  groundpath += "../../../res/ANYmal/robot.urdf";

  std::string robotpath(__FILE__);
  while (robotpath.back() != '/')
    robotpath.erase(robotpath.size() - 1, 1);
  robotpath += "../../../res/ANYmal/robot.urdf";

//  int planeUid = sim->loadURDF(groundpath);
//  printf("planeUid = %d\n", planeUid);

  int anymalid = sim->loadURDF(robotpath);
  printf("r2d2 #joints = %d\n", sim->getNumJoints(anymalid));
  printf("num body = %d\n", sim->getNumBodies());

  b3Vector3 basePosition = b3MakeVector3(0,0,1);
  b3Quaternion baseOrientation = b3Quaternion(0,0,0,1);

  sim->resetBasePositionAndOrientation(anymalid, basePosition, baseOrientation);
  sim->setGravity(b3MakeVector3(0,0,-9.8));

  while (sim->isConnected())
  {
    sim->stepSimulation();

    b3Vector3 pos;
    b3Quaternion quat;
    sim->getBasePositionAndOrientation(anymalid, pos, quat);

  }
  delete sim;
}
