//
// Created by kangd on 10.02.18.
//

#ifndef BULLETSIM_WORLD_HPP
#define BULLETSIM_WORLD_HPP

#include <btBulletDynamicsCommon.h>
#include <bulletSim/object/Sphere.hpp>

#include "Configure.hpp"
#include "bulletSim/object/Box.hpp"
#include "bulletSim/object/CheckerBoard.hpp"

namespace bullet_sim {

class World {

 public:
  explicit World();
  virtual ~World();

  object::Sphere *addSphere(double radius, double mass,
                            CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1);
  object::Box *addBox(double xLength, double yLength, double zLength, double mass,
                      CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1);
  object::CheckerBoard *addCheckerboard(double gridSize, double xLength, double yLength, double reflectanceI,
                                        CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1);

  void integrate(double dt);

  void setGravity(const btVector3 &gravity);

 private:

  // dynamics world
  btDiscreteDynamicsWorld* dynamicsWorld_;
  btBroadphaseInterface* broadphase_;
  btCollisionConfiguration* collisionConfiguration_;
  btCollisionDispatcher* collisionDispatcher_;

  // simulation properties
  btVector3 gravity_ = {0, 0, -9.81};

  // contact solver
  // TODO types and parameters
  btSequentialImpulseConstraintSolver* solver_;

  // list
  std::vector<object::SingleBodyObject*> objectList_;

//  std::vector<int> colIdxToObjIdx_;
//  std::vector<int> colIdxToLocalObjIdx_;

//  MaterialManager mat_;
//  MaterialPairProperties defaultMaterialProperty_;
//  double erp_ = 0.06;

};

}


#endif //BULLETSIM_WORLD_HPP
