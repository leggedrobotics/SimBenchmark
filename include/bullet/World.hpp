//
// Created by kangd on 10.02.18.
//

#ifndef BENCHMARK_WORLD_HPP
#define BENCHMARK_WORLD_HPP

#include <btBulletDynamicsCommon.h>
#include <configure.hpp>
#include "Box.hpp"

namespace bullet_sim {

class World {

//  friend class World_RG;

 public:
  explicit World();
  virtual ~World();

  object::Box *addBox(double xLength, double yLength, double zLength, double mass, CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1);

 protected:

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
//  std::vector<object::Object*> objectList_;
//  std::vector<contact::Single3DContactProblem> contactProblemList_;
//  std::vector<int> colIdxToObjIdx_;
//  std::vector<int> colIdxToLocalObjIdx_;

  // constraints
//  std::vector<StiffWire*> stiffWire_;
//  std::vector<CompliantWire*> compliantWire_;

//  MaterialManager mat_;
//  MaterialPairProperties defaultMaterialProperty_;
//  double erp_ = 0.06;

};

}


#endif //BENCHMARK_WORLD_HPP
