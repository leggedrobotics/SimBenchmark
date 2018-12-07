//
// Created by kangd on 24.05.18.
//


#include <raiCommon/utils/StopWatch.hpp>
#include "bullet/b3RobotSimulatorClientAPI_NoGUI.h"

b3RobotSimulatorClientAPI_NoGUI* sim;

void setJointConfig(int robotId) {
  int numJoint = sim->getNumJoints(robotId);
  std::vector<int> ctrbJoints;

  double jointPos[12] = {
      0.03, 0.4, -0.8,
      -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8,
      -0.03, -0.4, 0.8
  };

  int ctrbDof = 0;
  for (int i = 0; i < numJoint; i++) {
    b3JointInfo info;
    sim->getJointInfo(robotId, i, &info);

    switch(info.m_jointType) {
      case eFixedType:
        break;
      case eRevoluteType:
      case ePrismaticType: {
        ctrbJoints.push_back(i);
        ctrbDof++;
        break;
      }
      case eSphericalType:
      case ePlanarType:
      case ePoint2PointType:
      case eGearType:
      default:
        break;
    }
  }

  for(int i = 0; i < ctrbDof; i++) {
    b3RobotSimulatorJointMotorArgs arg(CONTROL_MODE_POSITION_VELOCITY_PD);
    arg.m_targetPosition = jointPos[i];
    arg.m_targetVelocity = 0;
    sim->setJointMotorControl(robotId, ctrbJoints[i], arg);
    sim->resetJointState(robotId, ctrbJoints[i], jointPos[i]);
  }
}

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

  std::string groundpath(__FILE__);
  while (groundpath.back() != '/')
    groundpath.erase(groundpath.size() - 1, 1);
  groundpath += "../../../res/bullet/Plane/plane.urdf";

  std::string robotpath(__FILE__);
  while (robotpath.back() != '/')
    robotpath.erase(robotpath.size() - 1, 1);
  robotpath += "../../../res/ANYmal/robot.urdf";

  int planeUid = sim->loadURDF(groundpath);
  printf("planeUid = %d\n", planeUid);

  // numrow
  int numrow = 1;
  if(argc == 2)
    numrow = atoi(argv[1]);
  printf("numrow: %d\n", numrow);

  for(int i = 0; i < numrow; i++) {
    for (int j = 0; j < numrow; j++) {
      b3RobotSimulatorLoadUrdfFileArgs arg;
      arg.m_flags = URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION | URDF_USE_IMPLICIT_CYLINDER | MJCF_COLORS_FROM_FILE;
      int robot = sim->loadURDF(robotpath, arg);
      setJointConfig(robot);

      btVector3 basePosition = btVector3(
          2.0 * i,
          2.0 * j,
          0.6);
      btQuaternion baseOrientation = btQuaternion(0,0,0,1);

      sim->resetBasePositionAndOrientation(robot, basePosition, baseOrientation);
    }
  }

  sim->setGravity(btVector3(0,0,-9.81));

  // parameter
  b3RobotSimulatorSetPhysicsEngineParameters params;
  params.m_defaultContactERP = 0.0;
  params.m_defaultNonContactERP = 0.0;
  params.m_frictionERP = 0.0;
  params.m_deltaTime = 0.0005;
  params.m_solverResidualThreshold = 1e-2;
  sim->setPhysicsEngineParameter(params);

  // loop
  const int numiter = 50000;
  StopWatch watch;
  watch.start();
  for (int i = 0; i < numiter && sim->isConnected(); i++)
  {
    sim->stepSimulation();
  }
  double time = watch.measure();

  // num contacts
  b3RobotSimulatorGetContactPointsArgs arg;
  b3ContactInformation info;
  sim->getContactPoints(arg, &info);
  printf("num contacts = %d\n", info.m_numContactPoints);

  // time
  printf("%d step takes %f sec", numiter, time);
  printf(" -> %f ms/step = %f kHz\n", time / numiter * 1000.0, numiter / time / 1000);

  delete sim;
}
