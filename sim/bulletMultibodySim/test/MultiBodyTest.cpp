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

  std::string groundpath(__FILE__);
  while (groundpath.back() != '/')
    groundpath.erase(groundpath.size() - 1, 1);
  groundpath += "../../../res/bullet/Plane/robot.urdf";

  std::string robotpath(__FILE__);
  while (robotpath.back() != '/')
    robotpath.erase(robotpath.size() - 1, 1);
  robotpath += "../../../res/ANYmal/robot.urdf";

  int planeUid = sim->loadURDF(groundpath);
  printf("planeUid = %d\n", planeUid);

  int robot = sim->loadURDF(robotpath);
  printf("robot #joints = %d\n", sim->getNumJoints(robot));
  printf("num body = %d\n", sim->getNumBodies());

  btVector3 basePosition = btVector3(0,0,1);
  btQuaternion baseOrientation = btQuaternion(0,0,0,1);

  sim->resetBasePositionAndOrientation(robot, basePosition, baseOrientation);
  sim->setGravity(btVector3(0,0,-9.81));

  b3VisualShapeInformation info;
  sim->getVisualShapeData(robot, info);

  while (sim->isConnected())
  {
    sim->stepSimulation();

    btVector3 pos;
    btQuaternion quat;
    sim->getBasePositionAndOrientation(robot, pos, quat);

  }
  delete sim;
}
