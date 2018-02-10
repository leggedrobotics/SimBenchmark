//
// Created by kangd on 10.02.18.
//

#include "World.hpp"

bullet_sim::World::World() {

  // broadphase
  broadphase_ = new btDbvtBroadphase();

  // collision configuration and dispatcher
  collisionConfiguration_ = new btDefaultCollisionConfiguration();
  collisionDispatcher_ = new btCollisionDispatcher(collisionConfiguration_);

  // physics solver
  solver_ = new btSequentialImpulseConstraintSolver;

  // world
  dynamicsWorld_ = new btDiscreteDynamicsWorld(collisionDispatcher_,
                                               broadphase_,
                                               solver_,
                                               collisionConfiguration_);
  dynamicsWorld_->setGravity(gravity_);
}

bullet_sim::World::~World() {
  // clean up
  delete dynamicsWorld_;
  delete solver_;
  delete collisionDispatcher_;
  delete collisionConfiguration_;
  delete broadphase_;

  // TODO object remove
}
bullet_sim::object::Box *bullet_sim::World::addBox(double xLength,
                                                   double yLength,
                                                   double zLength,
                                                   double mass,
                                                   CollisionGroupType collisionGroup,
                                                   CollisionGroupType collisionMask) {

  btCollisionShape* shape = new btBoxShape(btVector3(xLength * 0.5,
                                                        yLength * 0.5,
                                                        zLength * 0.5));
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(btVector3(0, 0, 0));
  btDefaultMotionState* motionState = new btDefaultMotionState(transform);
  btRigidBody* rigidBody = new btRigidBody(mass, motionState, shape);
  dynamicsWorld_->addRigidBody(rigidBody);

  return nullptr;
}


