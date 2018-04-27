//
// Created by kangd on 16.03.18.
//

#ifndef DARTSIM_WORLD_HPP
#define DARTSIM_WORLD_HPP

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

class DartWorld: benchmark::WorldInterface {

 public:
  explicit DartWorld(SolverOption solverOption = SOLVER_LCP_DANTZIG);
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

//  const std::vector<Single3DContactProblem> *getCollisionProblem() const;
  void setGravity(const benchmark::Vec<3> &gravity) override ;
  int getNumObject() override ;

  void setTimeStep(double timeStep);
//  void setERP(double erp, double erp2, double frictionErp);

 private:
  void integrate(double dt) override ;

  dart::simulation::WorldPtr dynamicsWorld_;

  // simulation properties
  Eigen::Vector3d gravity_ = {0, 0, -9.81};
  double timeStep_ = 0.01;

  // list
  std::vector<object::DartObject*> objectList_;

  // solver option
  SolverOption solverOption_ = SOLVER_LCP_DANTZIG;

};

} // dart_sim

#endif //DARTSIM_WORLD_HPP
