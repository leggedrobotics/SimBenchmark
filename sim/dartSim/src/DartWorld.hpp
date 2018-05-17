//
// Created by kangd on 16.03.18.
//

#ifndef DARTSIM_WORLD_HPP
#define DARTSIM_WORLD_HPP

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

#include "common/interface/WorldInterface.hpp"
#include "common/Configure.hpp"

#include "object/DartSphere.hpp"
#include "object/DartBox.hpp"
#include "object/DartCheckerBoard.hpp"
#include "object/DartCylinder.hpp"
#include "object/DartCapsule.hpp"
#include "object/DartArticulatedSystem.hpp"

namespace dart_sim {

enum SolverOption {
  SOLVER_LCP_DANTZIG,
  SOLVER_LCP_PGS
};

enum CollisionDetectorOption {
  COLLISION_DETECTOR_FCL,
  COLLISION_DETECTOR_DART,
  COLLISION_DETECTOR_BULLET,
  COLLISION_DETECTOR_ODE
};

struct Single3DContactProblem {
 public:
  Single3DContactProblem(const Eigen::Vector3d &pos,
                         const Eigen::Vector3d &normal,
                         const Eigen::Vector3d &force) {
    pos_.e() = pos;
    normal_.e() = normal;
    force_.e() = force;
  }

  benchmark::Vec<3> pos_;
  benchmark::Vec<3> normal_;
  benchmark::Vec<3> force_;
};

class DartWorld: benchmark::WorldInterface {

 public:
  explicit DartWorld(SolverOption solverOption = SOLVER_LCP_DANTZIG,
                     CollisionDetectorOption detectorOption = COLLISION_DETECTOR_FCL);
  virtual ~DartWorld();

  object::DartSphere *addSphere(double radius,
                                double mass,
                                benchmark::CollisionGroupType collisionGroup=1,
                                benchmark::CollisionGroupType collisionMask=-1) override ;

  object::DartBox *addBox(double xLength,
                          double yLength,
                          double zLength,
                          double mass,
                          benchmark::CollisionGroupType collisionGroup=1,
                          benchmark::CollisionGroupType collisionMask=-1) override ;

  object::DartCheckerBoard *addCheckerboard(double gridSize,
                                            double xLength,
                                            double yLength,
                                            double reflectanceI,
                                            bo::CheckerboardShape shape = bo::BOX_SHAPE,
                                            benchmark::CollisionGroupType collisionGroup=1,
                                            benchmark::CollisionGroupType collisionMask=-1) override ;

  object::DartCapsule *addCapsule(double radius,
                                  double height,
                                  double mass,
                                  benchmark::CollisionGroupType collisionGroup=1,
                                  benchmark::CollisionGroupType collisionMask=-1) override ;

  object::DartCylinder *addCylinder(double radius,
                                    double height,
                                    double mass,
                                    benchmark::CollisionGroupType collisionGroup=1,
                                    benchmark::CollisionGroupType collisionMask=-1) override ;

  object::DartArticulatedSystem *addArticulatedSystem(std::string urdfPath,
                                                      benchmark::CollisionGroupType collisionGroup=1,
                                                      benchmark::CollisionGroupType collisionMask=-1);


  void integrate();

  void setGravity(const benchmark::Vec<3> &gravity) override ;
  void setTimeStep(double timeStep);
  void setMaxContacts(int maxcontacts);

  int getNumObject() override ;
  const std::vector<Single3DContactProblem> &getCollisionProblem();
//  void setERP(double erp, double erp2, double frictionErp);

 private:
  void integrate(double dt) override ;
  void integrate1(double dt) override;
  void integrate2(double dt) override;

  dart::simulation::WorldPtr dynamicsWorld_;

  // simulation properties
  Eigen::Vector3d gravity_ = {0, 0, -9.81};
  double timeStep_ = 0.01;

  // list
  std::vector<object::DartObject*> objectList_;

  // collision problem list
  std::vector<Single3DContactProblem> contactProblemList_;

  // solver option
  SolverOption solverOption_ = SOLVER_LCP_DANTZIG;

};

} // dart_sim

#endif //DARTSIM_WORLD_HPP
