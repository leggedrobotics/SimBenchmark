//
// Created by kangd on 11.02.18.
//

#ifndef ODESIM_WORLD_HPP
#define ODESIM_WORLD_HPP

#include <ode/ode.h>

#include "interface/WorldInterface.hpp"
#include "odeSim/object/Sphere.hpp"
#include "odeSim/object/Box.hpp"
#include "odeSim/object/CheckerBoard.hpp"

namespace ode_sim {

class World: benchmark::WorldInterface {

 public:
  explicit World();
  virtual ~World();

  object::Sphere *addSphere(double radius, double mass,
                            CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1) override ;
  object::Box *addBox(double xLength, double yLength, double zLength, double mass,
                      CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1) override ;
  object::CheckerBoard *addCheckerboard(double gridSize, double xLength, double yLength, double reflectanceI,
                                        CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1) override ;

  void integrate(double dt) override ;

  void setGravity(const dVector3 &gravity);

  // dynamics world
  static dWorldID dynamicsWorld_;
  static dJointGroupID contactGroup_;

  // constants
  static const int maxContactsPerBody = 10;

 private:

  // call back
  static void nearCallback(void *data, dGeomID o1, dGeomID o2);

  // space
  dSpaceID space_;

  // simulation properties
  dVector3 gravity_ = {0, 0, -9.81};

  // contact solver
  // TODO types and parameters

  // list
  std::vector<object::SingleBodyObject*> objectList_;

};

}

#endif //ODESIM_WORLD_HPP
