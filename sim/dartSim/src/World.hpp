//
// Created by kangd on 16.03.18.
//

#ifndef DARTSIM_WORLD_HPP
#define DARTSIM_WORLD_HPP

#include "common/interface/WorldInterface.hpp"
#include "common/Configure.hpp"

#include "object/Sphere.hpp"
#include "object/Box.hpp"
#include "object/CheckerBoard.hpp"
#include "object/Cylinder.hpp"
#include "object/Capsule.hpp"

namespace dart_sim {

class World: benchmark::WorldInterface {

 public:
  explicit World();
  virtual ~World();

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

  object::CheckerBoard *addCheckerboard(double gridSize,
                                        double xLength,
                                        double yLength,
                                        double reflectanceI,
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

  void integrate();

//  const std::vector<Single3DContactProblem> *getCollisionProblem() const;
  void setGravity(const benchmark::Vec<3> &gravity) override ;

  void setTimeStep(double timeStep);
//  void setERP(double erp, double erp2, double frictionErp);

 private:
  void integrate(double dt) override ;

  dart::simulation::WorldPtr dynamicsWorld_;

  // simulation properties
  Eigen::Vector3d gravity_ = {0, 0, -9.81};
  double timeStep_ = 0.01;

  // list
  std::vector<object::SingleBodyObject*> objectList_;

};

} // dart_sim

#endif //DARTSIM_WORLD_HPP
