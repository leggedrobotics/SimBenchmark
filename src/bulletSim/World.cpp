//
// Created by kangd on 10.02.18.
//

#include "bulletSim/World.hpp"

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

  // remove objects
  for (auto *ob: objectList_)
    delete ob;

  // remove world
  delete dynamicsWorld_;
  delete solver_;
  delete collisionDispatcher_;
  delete collisionConfiguration_;
  delete broadphase_;
}

bullet_sim::object::Sphere *bullet_sim::World::addSphere(double radius,
                                                         double mass,
                                                         CollisionGroupType collisionGroup,
                                                         CollisionGroupType collisionMask) {
  object::Sphere *sphere = new bullet_sim::object::Sphere(radius, mass);
  dynamicsWorld_->addRigidBody(sphere->getRigidBody_());
  objectList_.push_back(sphere);
  return sphere;
}

bullet_sim::object::Box *bullet_sim::World::addBox(double xLength,
                                                   double yLength,
                                                   double zLength,
                                                   double mass,
                                                   CollisionGroupType collisionGroup,
                                                   CollisionGroupType collisionMask) {

  object::Box *box = new bullet_sim::object::Box(xLength, yLength, zLength, mass);
  dynamicsWorld_->addRigidBody(box->getRigidBody_());
  objectList_.push_back(box);
  return box;
}

bullet_sim::object::CheckerBoard *bullet_sim::World::addCheckerboard(double gridSize,
                                                                     double xLength,
                                                                     double yLength,
                                                                     double reflectanceI,
                                                                     CollisionGroupType collisionGroup,
                                                                     CollisionGroupType collisionMask) {

  object::CheckerBoard *checkerBoard = new bullet_sim::object::CheckerBoard();
  dynamicsWorld_->addRigidBody(checkerBoard->getRigidBody_());
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

void bullet_sim::World::integrate(double dt) {
  // TODO substep
  dynamicsWorld_->stepSimulation(dt, 1);
}

void bullet_sim::World::setGravity(const btVector3 &gravity_) {
  World::gravity_ = gravity_;
  dynamicsWorld_->setGravity(gravity_);
}



