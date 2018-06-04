//
// Created by kangd on 24.05.18.
//


#include <raiCommon/utils/StopWatch.hpp>
#include "api/b3RobotSimulatorClientAPI_NoGUI.h"

b3RobotSimulatorClientAPI_NoGUI* sim;

int main(int argc, char* argv[])
{
  sim = new b3RobotSimulatorClientAPI_NoGUI();

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
  sim->setGravity(btVector3(0,0,-9.81));

  // parameter
  b3RobotSimulatorSetPhysicsEngineParameters params;
  params.m_erp = 0.0;
  params.m_contactERP = 0.0;
  params.m_frictionERP = 0.0;
  params.m_fixedTimeStep = 0.0005;
//  params.m_solverResidualThreshold = 1e-4;
  sim->setPhysicsEngineParameter(params);

  // add objects
  std::string groundpath(__FILE__);
  while (groundpath.back() != '/')
    groundpath.erase(groundpath.size() - 1, 1);
  groundpath += "../../../res/bullet/Plane/plane.urdf";

  std::string robotpath(__FILE__);
  while (robotpath.back() != '/')
    robotpath.erase(robotpath.size() - 1, 1);
  robotpath += "../../../res/bullet/Ball/ball.urdf";

  b3RobotSimulatorLoadUrdfFileArgs urdfArgPlane;
  urdfArgPlane.m_flags = URDF_USE_INERTIA_FROM_FILE;
  int planeUid = sim->loadURDF(groundpath, urdfArgPlane);
  printf("planeUid = %d\n", planeUid);

  b3RobotSimulatorLoadUrdfFileArgs urdfArgBall;
  urdfArgBall.m_flags = URDF_USE_INERTIA_FROM_FILE;
  int ballUid = sim->loadURDF(robotpath, urdfArgBall);

  btVector3 basePosition = btVector3(0., 0., 5.0);
  btQuaternion baseOrientation = btQuaternion(0,0,0,1);
  sim->resetBasePositionAndOrientation(ballUid, basePosition, baseOrientation);

  // dynamics
  // ground
  b3RobotSimulatorChangeDynamicsArgs dynamicsArgPlane;
  dynamicsArgPlane.m_restitution = 1.0;
  dynamicsArgPlane.m_angularDamping = 0.0;
  dynamicsArgPlane.m_linearDamping = 0.0;
  dynamicsArgPlane.m_lateralFriction = 0.0;
  dynamicsArgPlane.m_rollingFriction = 0.0;
  dynamicsArgPlane.m_spinningFriction = 0.0;
  sim->changeDynamics(planeUid, -1, dynamicsArgPlane);

  // ball
  b3RobotSimulatorChangeDynamicsArgs dynamicsArgsBall;
  dynamicsArgsBall.m_restitution = 1.0;
  dynamicsArgsBall.m_angularDamping = 0.0;
  dynamicsArgsBall.m_linearDamping = 0.0;
  dynamicsArgsBall.m_lateralFriction = 0.0;
  dynamicsArgsBall.m_rollingFriction = 0.0;
  dynamicsArgsBall.m_spinningFriction = 0.0;
  sim->changeDynamics(ballUid, -1, dynamicsArgsBall);


  // loop
  const int numiter = 50000;
  StopWatch watch;
  watch.start();

  btVector3 position;
  btQuaternion quaternion;
  for (int i = 0; i < numiter && sim->isConnected(); i++)
  {
    sim->getBasePositionAndOrientation(ballUid, position, quaternion);
    printf("z = %f\n", position.z());
    sim->stepSimulation();
  }
  double time = watch.measure();

  // time
  printf("%d step takes %f sec", numiter, time);
  printf(" -> %f ms/step = %f kHz\n", time / numiter * 1000.0, numiter / time / 1000);

  delete sim;
}
