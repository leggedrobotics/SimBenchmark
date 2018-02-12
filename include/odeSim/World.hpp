//
// Created by kangd on 11.02.18.
//

#ifndef ODESIM_WORLD_HPP
#define ODESIM_WORLD_HPP

#include <ode/ode.h>

#include "Configure.hpp"
#include "odeSim/object/Box.hpp"
#include "odeSim/object/CheckerBoard.hpp"

namespace ode_sim {

class World {

 public:
  explicit World();
  virtual ~World();

  object::Box *addBox(double xLength, double yLength, double zLength, double mass,
                      CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1);
  object::CheckerBoard *addCheckerboard(double gridSize, double xLength, double yLength, double reflectanceI,
                                        CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1);

  void integrate(double dt);

  void setGravity(const dVector3 &gravity);

 private:

  // dynamics world
  dWorldID dynamicsWorld_;
  dSpaceID space_;
  dJointGroupID jointGroup_;

  // simulation properties
  dVector3 gravity_ = {0, 0, -9.81};

  // contact solver
  // TODO types and parameters

  // list
  std::vector<object::SingleBodyObject*> objectList_;


};

}


#endif //ODESIM_WORLD_HPP
