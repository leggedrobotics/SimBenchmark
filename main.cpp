#include <iostream>
#include <btBulletDynamicsCommon.h>

int main() {
  std::cout << "Hello, World!" << std::endl;

  const btScalar gravity = -9.81;

  // broadphase
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  // collision configuration and dispatcher
  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  // physics solver
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

  // world.
  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, gravity, 0));

  // objects
  // ground
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
  btTransform groundTransform;
  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0, -1, 0));

  btScalar groundMass = 0.0;
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);

  btRigidBody* ground = new btRigidBody(groundMass, groundMotionState, groundShape);
  dynamicsWorld->addRigidBody(ground);

//  btBoxShape* boxShape1 =
//  auto box1 = raiSim.addBox(1, 1, 1, 100);
//  box1->setPosition(0, 0, 0.5);
//  auto box2 = raiSim.addBox(1, 1, 1, 100);
//  box2->setPosition(0, 0, 1.5);
//  auto box3 = raiSim.addBox(1, 1, 1, 100);
//  box3->setPosition(0, 0, 2.5);
//  auto box4 = raiSim.addBox(1, 1, 1, 100);
//  box4->setPosition(0, 0, 3.5);
//

  for (int i = 0; i < 1000; i++) {
    // todo check max sub time step
    dynamicsWorld->stepSimulation(0.01, 1);
  }


  return 0;
}