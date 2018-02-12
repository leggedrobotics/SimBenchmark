//
// Created by kangd on 11.02.18.
//

#include "odeSim/World.hpp"

// collision detecting callback
static void nearCallback(void *contact, dGeomID o1, dGeomID o2) {
  auto *contacts = static_cast<std::pair<std::vector<dContactGeom>, int> *>(contact);
  if (contacts->first.size() < contacts->second + 500)
    contacts->first.resize(contacts->first.size() * 3);

  contacts->second += dCollide(o1, o2, 500, &contacts->first[contacts->second], sizeof(dContactGeom));
}

ode_sim::World::World() {

  // world
  dInitODE();
  dynamicsWorld_ = dWorldCreate();

  dVector3 Center = {0, 0, 0, 0};
  dVector3 Extents = {10, 0, 10, 0};
  space_ = dQuadTreeSpaceCreate(0, Center, Extents, 7);
  jointGroup_ = dJointGroupCreate(0);

  dWorldSetGravity(dynamicsWorld_, gravity_[0], gravity_[1], gravity_[2]);

//  dWorldSetCFM(world,1e-5);
//  dWorldSetAutoDisableFlag(dynamicsWorld_, 1);

#if 1
//  dWorldSetAutoDisableAverageSamplesCount(dynamicsWorld_, 10);
#endif

//  dWorldSetLinearDamping(world, 0.00001);
//  dWorldSetAngularDamping(world, 0.005);
//  dWorldSetMaxAngularSpeed(world, 200);
//  dWorldSetContactMaxCorrectingVel(world,0.1);
//  dWorldSetContactSurfaceLayer(world,0.001);

}

ode_sim::World::~World() {

  // remove objects
  for (auto *ob: objectList_)
    delete ob;

  // remove world
  dJointGroupDestroy(jointGroup_);
  dSpaceDestroy(space_);
  dWorldDestroy(dynamicsWorld_);
  dCloseODE();
}

void ode_sim::World::setGravity(const dVector3 &gravity) {
  memcpy(gravity_, gravity, sizeof(dVector3));
  dWorldSetGravity(dynamicsWorld_, gravity_[0], gravity_[1], gravity_[2]);
}

void ode_sim::World::integrate(double dt) {

  // collision detection
  dSpaceCollide(space_, 0, &nearCallback);

  // collision solving
//  if (!pause) {
//    if (solver == 0)
//      dWorldQuickStep(dynamicsWorld_, dt);
//    else
      dWorldStep(dynamicsWorld_, dt);
//  }
}

ode_sim::object::Box *ode_sim::World::addBox(double xLength,
                                             double yLength,
                                             double zLength,
                                             double mass,
                                             CollisionGroupType collisionGroup,
                                             CollisionGroupType collisionMask) {
  object::Box *box = new ode_sim::object::Box(xLength, yLength, zLength, mass, dynamicsWorld_, space_);
  objectList_.push_back(box);
  return box;
}

ode_sim::object::CheckerBoard *ode_sim::World::addCheckerboard(double gridSize,
                                                               double xLength,
                                                               double yLength,
                                                               double reflectanceI,
                                                               CollisionGroupType collisionGroup,
                                                               CollisionGroupType collisionMask) {
  object::CheckerBoard *checkerBoard = new ode_sim::object::CheckerBoard(dynamicsWorld_, space_);
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}
