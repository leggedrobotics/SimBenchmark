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
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>

#include "common/Configure.hpp"
#include "common/interface/WorldInterface.hpp"

#include "object/Object.hpp"
#include "object/CheckerBoard.hpp"
#include "object/Sphere.hpp"
#include "object/Cylinder.hpp"
#include "object/Box.hpp"
#include "object/Capsule.hpp"
#include "object/ArticulatedSystem/ArticulatedSystem.hpp"

namespace bullet_sim {

enum SolverOption {
  SOLVER_SEQUENTIAL_IMPULSE,
  SOLVER_NNCG,
  SOLVER_MLCP_PGS,
  SOLVER_MLCP_DANTZIG,
  SOLVER_MLCP_LEMKE,
  SOLVER_MULTI_BODY
};

struct Single3DContactProblem {
  Single3DContactProblem(const btVector3 &point, const btVector3 &normal) {
    point_ = {point.x(), point.y(), point.z()};
    normal_ = {normal.x(), normal.y(), normal.z()};
  };
  Eigen::Vector3d point_;
  Eigen::Vector3d normal_;
};

class World: public benchmark::WorldInterface {

 public:
  explicit World(SolverOption solverOption = SOLVER_SEQUENTIAL_IMPULSE);
  virtual ~World();

  object::CheckerBoard *addCheckerboard(double gridSize,
                                        double xLength,
                                        double yLength,
                                        double reflectanceI,
                                        bo::CheckerboardShape shape = bo::PLANE_SHAPE,
                                        benchmark::CollisionGroupType collisionGroup=1,
                                        benchmark::CollisionGroupType collisionMask=-1) override;

  object::Sphere *addSphere(double radius,
                            double mass,
                            benchmark::CollisionGroupType collisionGroup=1,
                            benchmark::CollisionGroupType collisionMask=-1) override ;

  object::Box *addBox(double xLength,
                      double yLength,
                      double zLength,
                      double mass,
                      benchmark::CollisionGroupType collisionGroup=1,
                      benchmark::CollisionGroupType collisionMask=-1) override ;

  object::Capsule *addCapsule(double radius,
                              double height,
                              double mass,
                              benchmark::CollisionGroupType collisionGroup=1,
                              benchmark::CollisionGroupType collisionMask=-1) override ;

  object::Cylinder *addCylinder(double radius,
                                double height,
                                double mass,
                                benchmark::CollisionGroupType collisionGroup=1,
                                benchmark::CollisionGroupType collisionMask=-1) override ;

  object::ArticulatedSystem *addArticulatedSystem(std::string urdfPath,
                                                  benchmark::CollisionGroupType collisionGroup=1,
                                                  benchmark::CollisionGroupType collisionMask=-1);

  void integrate(double dt) override ;

  const std::vector<Single3DContactProblem> *getCollisionProblem() const;

  void setGravity(const benchmark::Vec<3> &gravity) override ;

  void setERP(double erp, double erp2, double frictionErp);

  int getNumObject() override ;

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
