//
// Created by kangd on 18.02.18.
//

#ifndef BENCHMARK_WORLD_HPP
#define BENCHMARK_WORLD_HPP

#include <mujoco.h>

#include "common/Configure.hpp"
#include "common/interface/WorldInterface.hpp"

#include "object/Sphere.hpp"
#include "object/Box.hpp"
#include "object/Capsule.hpp"
#include "object/CheckerBoard.hpp"
#include "object/Cylinder.hpp"

namespace mujoco_sim {

typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;
typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

enum SolverOption {
  SOLVER_PGS,
  SOLVER_CG,
  SOLVER_NEWTON
};

class World: public benchmark::WorldInterface {

 public:
  World(const char *modelPath,
          const char *keyPath,
          SolverOption solverOption);
  virtual ~World();

  /// note: use last two parameters as bodyId and geomId rather than collisionGroup and collisionMask
  object::Sphere *addSphere(double radius,
                            double mass,
                            int bodyId,
                            int geomId) override ;

  object::Box *addBox(double xLength,
                      double yLength,
                      double zLength,
                      double mass,
                      int bodyId,
                      int geomId) override;

  object::CheckerBoard *addCheckerboard(double gridSize,
                                        double xLength,
                                        double yLength,
                                        double reflectanceI,
                                        bo::CheckerboardShape shape,
                                        int bodyId,
                                        int geomId) override;

  object::Capsule *addCapsule(double radius,
                              double height,
                              double mass,
                              int bodyId,
                              int geomId) override;

  object::Cylinder *addCylinder(double radius,
                                double height,
                                double mass,
                                int bodyId,
                                int geomId) override;

  void setGravity(const benchmark::Vec<3> &gravity) override ;

  mjModel *getWorldModel() const;
  mjData *getWorldData() const;
  int getWorldNumContacts();
  int getNumObject() override ;

  void integrate(double dt);

  /// the functions below are articulated system related.
  /// ===================================
  const EigenVec getGeneralizedCoordinate();
  const EigenVec getGeneralizedVelocity();
  /* For floating-base robots, [linearPosition_W, baseRationInQuaternion, joint Angles]
   * For fixed-base robot, [joint angles]
   * The dimension is the DOF+1 for floating-based, and DOF for fixed based. (obtained by getDOF())*/
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState);
  /* For floating-base robots, [linearVelocity_W, angularVelocity_W, jointVelocity]
   * The dimension is the same as dof (obtained with getDOF)*/
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel);
  void setGeneralizedCoordinate(std::initializer_list<double> jointState);
  void setGeneralizedVelocity(std::initializer_list<double> jointVel);
  void setGeneralizedForce(std::initializer_list<double> tau);
  void setGeneralizedForce(const Eigen::VectorXd &tau);
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel);
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel);
  const EigenVec getGeneralizedForce();
  int getDOF();
  int getGeneralizedCoordinateDim();
  /// ===================================

 private:

  mjModel *worldModel_;
  mjData *worldData_;
  mjOption *simOption_;

  // list
  std::vector<object::SingleBodyObject*> objectList_;

  // generalized coordinate
  benchmark::VecDyn generalizedCoordinate_;
  benchmark::VecDyn generalizedVelocity_;
  benchmark::VecDyn generalizedForce_;

  // dim
  int dof_ = 0;
  int dimGenCoord_ = 0;
  int numActuators_ = 0;
};

} // mujoco_sim

#endif //BENCHMARK_WORLD_HPP
