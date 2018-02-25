//
// Created by kangd on 11.02.18.
//

#include "odeSim/World.hpp"

// static members
dWorldID ode_sim::World::dynamicsWorld_;
dJointGroupID ode_sim::World::contactGroup_;

void message(int errnum, const char *msg, va_list ap) {
  // no debug message
}

ode_sim::World::World(SolverOption solverOption) : solverOption_(solverOption) {

  // world
  dInitODE();
  dynamicsWorld_ = dWorldCreate();
  dSetMessageHandler(0);

  dVector3 Center = {0, 0, 0, 0};
  dVector3 Extents = {10, 0, 10, 0};
  space_ = dQuadTreeSpaceCreate(0, Center, Extents, 7);
  contactGroup_ = dJointGroupCreate(0);

  dWorldSetGravity(dynamicsWorld_, gravity_[0], gravity_[1], gravity_[2]);

////  auto disable
//  dWorldSetAutoDisableAverageSamplesCount(dynamicsWorld_, 10);
//  dWorldSetAutoDisableFlag(dynamicsWorld_, 1);

////  parameters
//  dWorldSetCFM(dynamicsWorld_, 1e-5);
//  dWorldSetLinearDamping(dynamicsWorld_, 0.00001);
//  dWorldSetAngularDamping(world, 0.005);
//  dWorldSetMaxAngularSpeed(world, 200);
//  dWorldSetContactMaxCorrectingVel(world,0.1);
//  dWorldSetContactSurfaceLayer(world,0.001);
  dWorldSetERP(dynamicsWorld_, 0);
  dWorldSetCFM(dynamicsWorld_, 0);

  // no debug message
  dSetMessageHandler(message);
}

ode_sim::World::~World() {

  // remove objects
  for (auto *ob: objectList_)
    delete ob;

  // remove world
  dJointGroupDestroy(contactGroup_);
  dSpaceDestroy(space_);
  dWorldDestroy(dynamicsWorld_);
  dCloseODE();
}

void ode_sim::World::setGravity(const dVector3 &gravity) {
  memcpy(gravity_, gravity, sizeof(dVector3));
  dWorldSetGravity(dynamicsWorld_, gravity_[0], gravity_[1], gravity_[2]);
}


// collision detecting callback
void ode_sim::World::nearCallback(void *data, dGeomID o1, dGeomID o2) {
  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
    return;

  dContact contact[maxContactsPerBody];   // up to MAX_CONTACTS contacts per box-box

  for (i=0; i<maxContactsPerBody; i++) {
    object::MetrialProp *prop1 = (object::MetrialProp*)dGeomGetData(o1);
    object::MetrialProp *prop2 = (object::MetrialProp*)dGeomGetData(o2);

    contact[i].surface.mode = dContactBounce | dContactApprox1;
    contact[i].surface.mu = prop1->frictionalCoeff * prop2->frictionalCoeff;
    contact[i].surface.bounce = prop1->restitutionCoeff * prop2->restitutionCoeff;
  }
  if (int numc = dCollide(o1,o2, maxContactsPerBody, &contact[0].geom,
                          sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (dynamicsWorld_, contactGroup_, contact+i);
      dJointAttach (c,b1,b2);

//      if (show_contacts) {
//        dsSetColor(0,0,1);
//        dsDrawBox(contact[i].geom.pos,RI,ss);
//      }
    }
  }
}

void ode_sim::World::integrate(double dt) {

  // collision detection
  dSpaceCollide(space_, 0, &nearCallback);

  // collision solving
  if(solverOption_ == SOLVER_QUICK)
  {
    dWorldQuickStep(dynamicsWorld_, dt);
  }
  else
  {
    dWorldStep(dynamicsWorld_, dt);
  }

  dJointGroupEmpty(contactGroup_);
}

ode_sim::object::Sphere *ode_sim::World::addSphere(double radius,
                                                   double mass,
                                                   benchmark::CollisionGroupType collisionGroup,
                                                   benchmark::CollisionGroupType collisionMask) {
  object::Sphere *sphere = new ode_sim::object::Sphere(radius, mass, dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(sphere);
  return sphere;
}

ode_sim::object::Box *ode_sim::World::addBox(double xLength,
                                             double yLength,
                                             double zLength,
                                             double mass,
                                             benchmark::CollisionGroupType collisionGroup,
                                             benchmark::CollisionGroupType collisionMask) {
  object::Box *box = new ode_sim::object::Box(xLength, yLength, zLength, mass, dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(box);
  return box;
}

ode_sim::object::Capsule *ode_sim::World::addCapsule(double radius,
                                                     double height,
                                                     double mass,
                                                     benchmark::CollisionGroupType collisionGroup,
                                                     benchmark::CollisionGroupType collisionMask) {
  object::Capsule *capsule = new ode_sim::object::Capsule(radius, height, mass, dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(capsule);
  return capsule;
}

ode_sim::object::CheckerBoard *ode_sim::World::addCheckerboard(double gridSize,
                                                               double xLength,
                                                               double yLength,
                                                               double reflectanceI,
                                                               benchmark::CollisionGroupType collisionGroup,
                                                               benchmark::CollisionGroupType collisionMask) {
  object::CheckerBoard *checkerBoard = new ode_sim::object::CheckerBoard(dynamicsWorld_, space_, collisionGroup, collisionMask);
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

void ode_sim::World::setERP(double erp) {
  dWorldSetERP(dynamicsWorld_, erp);
}
