//
// Created by kangd on 25.03.18.
//

#ifndef BENCHMARK_ARTICULATEDSYSTEM_HPP
#define BENCHMARK_ARTICULATEDSYSTEM_HPP

#include "../math.hpp"

namespace benchmark {
namespace object {

enum class Shape {
  Box = 0,
  Cylinder,
  Sphere,
  Mesh,
  Capsule,
  Cone
};

class ArticulatedSystemInterface {

 public:
  typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;
  typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

  std::vector<std::tuple<benchmark::Mat<3, 3>, benchmark::Vec<3>, int, Shape, benchmark::Vec<4>>>& getVisOb() {
    return visObj;
  };

  std::vector<std::tuple<benchmark::Mat<3, 3>, benchmark::Vec<3>, int, Shape>>& getVisColOb() {
    return visColObj;
  };

  virtual const EigenVec getGeneralizedCoordinate() = 0;

  virtual const EigenVec getGeneralizedVelocity() = 0;

  /* For floating-base robots, [linearPosition_W, baseRationInQuaternion, joint Angles]
   * For fixed-base robot, [joint angles]
   * The dimension is the DOF+1 for floating-based, and DOF for fixed based. (obtained by getDOF())*/
  virtual void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) = 0;

  /* For floating-base robots, [linearVelocity_W, angularVelocity_W, jointVelocity]
   * The dimension is the same as dof (obtained with getDOF)*/
  virtual void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) = 0;

  virtual void setGeneralizedCoordinate(std::initializer_list<double> jointState) = 0;

  virtual void setGeneralizedVelocity(std::initializer_list<double> jointVel) = 0;

  virtual void setGeneralizedForce(std::initializer_list<double> tau) = 0;

  virtual void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) = 0;

  virtual void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) = 0;

  virtual void setGeneralizedForce(const Eigen::VectorXd &tau) = 0;

  virtual const EigenVec getGeneralizedForce() = 0;

  virtual int getDOF() = 0;

 public:
  // orientation, position, link_id, shape, color
  std::vector<std::tuple<benchmark::Mat<3, 3>, benchmark::Vec<3>, int, Shape>> visColObj;
  std::vector<std::tuple<benchmark::Mat<3, 3>, benchmark::Vec<3>, int, Shape, benchmark::Vec<4>>> visObj;
  std::vector<std::pair<std::string, benchmark::Vec<4>>> visProps_;
  std::vector<std::pair<std::string, benchmark::Vec<4>>> visColProps_;

 protected:
  int dof_ = 0;
  int stateDimension_ = 0;

};

} // object
} // benchmark

#endif //BENCHMARK_ARTICULATEDSYSTEM_HPP
