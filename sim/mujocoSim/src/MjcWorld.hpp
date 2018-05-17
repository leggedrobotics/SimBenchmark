//
// Created by kangd on 18.02.18.
//

#ifndef BENCHMARK_WORLD_HPP
#define BENCHMARK_WORLD_HPP

#include <mujoco.h>

#include "common/Configure.hpp"
#include "common/interface/WorldInterface.hpp"

#include "object/MjcSphere.hpp"
#include "object/MjcBox.hpp"
#include "object/MjcCapsule.hpp"
#include "object/MjcCheckerBoard.hpp"
#include "object/MjcCylinder.hpp"

namespace mujoco_sim {

typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;
typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

enum SolverOption {
  SOLVER_PGS,
  SOLVER_CG,
  SOLVER_NEWTON
};

class MjcWorld: public benchmark::WorldInterface {

 public:
  MjcWorld(const char *modelPath,
          const char *keyPath,
          SolverOption solverOption);
  virtual ~MjcWorld();

  /// note: use last two parameters as bodyId and geomId rather than collisionGroup and collisionMask
  object::MjcSphere *addSphere(double radius,
                            double mass,
                            int bodyId,
                            int geomId) override ;

  object::MjcBox *addBox(double xLength,
                      double yLength,
                      double zLength,
                      double mass,
                      int bodyId,
                      int geomId) override;

  object::MjcCheckerBoard *addCheckerboard(double gridSize,
                                        double xLength,
                                        double yLength,
                                        double reflectanceI,
                                        bo::CheckerboardShape shape,
                                        int bodyId,
                                        int geomId) override;

  object::MjcCapsule *addCapsule(double radius,
                              double height,
                              double mass,
                              int bodyId,
                              int geomId) override;

  object::MjcCylinder *addCylinder(double radius,
                                double height,
                                double mass,
                                int bodyId,
                                int geomId) override;

  void setGravity(const benchmark::Vec<3> &gravity) override ;
  void setNoSlipParameter(int maxIter);
  void setTimeStep(double timeStep);

  mjModel *getWorldModel() const;
  mjData *getWorldData() const;

  int getWorldNumContacts();
  int getNumObject() override ;

  const Eigen::Map<Eigen::Matrix<double, 3, 1>> getLinearMomentumInCartesianSpace();
  double getTotalMass();

  void integrate();
  void integrate1();
  void integrate2();

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

  /// deprecated function
  void integrate(double dt) override ;
  void integrate1(double dt) override;
  void integrate2(double dt) override;

  mjModel *worldModel_;
  mjData *worldData_;
  mjOption *simOption_;

  // list
  std::vector<object::MjcSingleBodyObject*> objectList_;

  // generalized coordinate
  benchmark::VecDyn generalizedCoordinate_;
  benchmark::VecDyn generalizedVelocity_;
  benchmark::VecDyn generalizedForce_;

  // linear momentum
  benchmark::Vec<3> linearMomentum_;

  // dim
  int dof_ = 0;
  int dimGenCoord_ = 0;
  int numActuators_ = 0;
};

} // mujoco_sim

#endif //BENCHMARK_WORLD_HPP
