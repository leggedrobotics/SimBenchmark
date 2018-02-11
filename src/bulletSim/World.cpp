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

  return new bullet_sim::object::Box(xLength, yLength, zLength, mass);
}

bullet_sim::object::CheckerBoard *bullet_sim::World::addCheckerboard(double gridSize,
                                                                     double xLength,
                                                                     double yLength,
                                                                     double reflectanceI,
                                                                     CollisionGroupType collisionGroup,
                                                                     CollisionGroupType collisionMask) {

  return new bullet_sim::object::CheckerBoard();
}

void bullet_sim::World::integrate(double dt) {
  // TODO substep
  dynamicsWorld_->stepSimulation(dt, 1);
}


