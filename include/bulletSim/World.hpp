//
// Created by kangd on 10.02.18.
//

#ifndef BULLETSIM_WORLD_HPP
#define BULLETSIM_WORLD_HPP

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btLemkeSolver.h>
#include <Configure.hpp>

#include "bulletSim/object/Sphere.hpp"
#include "bulletSim/object/Box.hpp"
#include "bulletSim/object/Capsule.hpp"
#include "bulletSim/object/CheckerBoard.hpp"

namespace bullet_sim {

enum SolverOption {
  SOLVER_SEQUENTIAL_IMPULSE,
  SOLVER_NNCG,
  SOLVER_MLCP_PGS,
  SOLVER_MLCP_DANTZIG,
  SOLVER_MLCP_LEMKE
};

struct Single3DContactProblem {
  Single3DContactProblem(const btVector3 &point, const btVector3 &normal) {
    point_ = {point.x(), point.y(), point.z()};
    normal_ = {normal.x(), normal.y(), normal.z()};
  };
  Eigen::Vector3d point_;
  Eigen::Vector3d normal_;
};

class World {

 public:
  explicit World(SolverOption solverOption = SOLVER_SEQUENTIAL_IMPULSE);
  virtual ~World();

  object::Sphere *addSphere(double radius, double mass,
                            CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1) ;
  object::Box *addBox(double xLength, double yLength, double zLength, double mass,
                      CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1) ;
  object::CheckerBoard *addCheckerboard(double gridSize, double xLength, double yLength, double reflectanceI,
                                        CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1);
  object::Capsule *addCapsule(double radius, double height, double mass,
                              CollisionGroupType collisionGroup=1, CollisionGroupType collisionMask=-1);

  void integrate(double dt);

  const std::vector<Single3DContactProblem> *getCollisionProblem() const;
  void setGravity(const btVector3 &gravity);

 private:

  // dynamics world
  btDiscreteDynamicsWorld* dynamicsWorld_;
  btBroadphaseInterface* broadphase_;
  btDefaultCollisionConfiguration* collisionConfiguration_;
  btCollisionDispatcher* collisionDispatcher_;

  // simulation properties
  btVector3 gravity_ = {0, 0, -9.81};

  // contact solver
  SolverOption solverOption_ = SOLVER_SEQUENTIAL_IMPULSE;
  btConstraintSolver* solver_;
  btMLCPSolverInterface* mlcpSolver_ = 0;

  // list
  std::vector<object::Object*> objectList_;
  std::vector<Single3DContactProblem> contactProblemList_;

//  std::vector<int> colIdxToObjIdx_;
//  std::vector<int> colIdxToLocalObjIdx_;

//  MaterialManager mat_;
//  MaterialPairProperties defaultMaterialProperty_;
//  double erp_ = 0.06;

};

}


#endif //BULLETSIM_WORLD_HPP
