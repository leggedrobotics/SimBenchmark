/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
#include "btBulletDynamicsCommon.h"
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include "bulletSim/object/ArticulatedSystem/URDF/BulletUrdfImporter.h"
#include "bulletSim/object/ArticulatedSystem/URDF/MyMultiBodyCreator.h"
#include "bulletSim/object/ArticulatedSystem/URDF/UrdfParser.h"
#include "bulletSim/object/ArticulatedSystem/URDF/URDFToBullet.h"
#include <stdio.h>

/// This is a Hello World program for running a basic Bullet physics simulation

int main(int argc, char** argv)
{
  ///-----includes_end-----

  int i;
  ///-----initialization_start-----

  ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

  ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
  btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

  ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
  btMultiBodyConstraintSolver* solver = new btMultiBodyConstraintSolver();

  btDiscreteDynamicsWorld* dynamicsWorld = new btMultiBodyDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0, 0, 0));

  ///-----initialization_end-----

  //keep track of the shapes, we release memory at exit.
  //make sure to re-use collision shapes among rigid bodies whenever possible!
  btAlignedObjectArray<btCollisionShape*> collisionShapes;

  ///create a few basic rigid bodies

  //the ground is a cube of side 100 at position y = -56.
  //the sphere will hit it at y = -6, with center at -5
  {
    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

    collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -56, 0));

    btScalar mass(0.);

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      groundShape->calculateLocalInertia(mass, localInertia);

    //using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    //add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
  }

  /// Do some simulation
  BulletURDFImporter importer(0, 0, 1.0, 0);
  bool loadOK = importer.loadURDF("../res/ANYmal/robot.urdf");

  if(loadOK) {
    MyMultiBodyCreator creator(0);

    btTransform identityTrans;
    identityTrans.setIdentity();

    ConvertURDF2Bullet2(importer, creator, identityTrans, (btMultiBodyDynamicsWorld *)dynamicsWorld, true, importer.getPathPrefix());
  }

    ///-----stepsimulation_start-----
  for (i = 0; i < 150; i++)
  {
    dynamicsWorld->stepSimulation(1.f / 60.f, 0);

    //print positions of all objects
    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
    {
      btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
      btRigidBody* body = btRigidBody::upcast(obj);
      btTransform trans;
      if (body && body->getMotionState())
      {
        body->getMotionState()->getWorldTransform(trans);
      }
      else
      {
        trans = obj->getWorldTransform();
      }
      printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
    }
  }

  ///-----stepsimulation_end-----

  //cleanup in the reverse order of creation/initialization

  ///-----cleanup_start-----

  //remove the rigidbodies from the dynamics world and delete them
  for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
  {
    btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState())
    {
      delete body->getMotionState();
    }
    dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }

  //delete collision shapes
  for (int j = 0; j < collisionShapes.size(); j++)
  {
    btCollisionShape* shape = collisionShapes[j];
    collisionShapes[j] = 0;
    delete shape;
  }

  //delete dynamics world
  delete dynamicsWorld;

  //delete solver
  delete solver;

  //delete broadphase
  delete overlappingPairCache;

  //delete dispatcher
  delete dispatcher;

  delete collisionConfiguration;

  //next line is optional: it will be cleared by the destructor when the array goes out of scope
  collisionShapes.clear();
}