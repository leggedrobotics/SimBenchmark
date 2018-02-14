//
// Created by kangd on 13.02.18.
//

#ifndef BULLETSIM_ARTICULATEDSYSTEM_HPP
#define BULLETSIM_ARTICULATEDSYSTEM_HPP

#include <iostream>
#include <fstream>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Geometry>
#include <raiSim/math.hpp>

#include "Object.hpp"

namespace bullet_sim {
namespace object {

enum class Shape {
  Box = 0,
  Cylinder,
  Sphere,
  Mesh,
  Capsule,
  Cone
};

class ArticulatedSystem: public Object {

 public:

  typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;
  typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

  explicit ArticulatedSystem(std::string urdfFile, std::vector<std::string> jointOrder = std::vector<std::string>());

  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape, rai_sim::Vec<4>>>& getVisOb() {
    return visObj;
  };

  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape>>& getVisColOb() {
    return visColObj;
  };

 private:

//  void init();

//  void computeExpandedParentArray();

//  bool processLinkFromUrdf(boost::shared_ptr<const urdf::Link> urdfLink, Child &raiLink, std::vector<std::string> &jointsOrder);

 protected:
//  std::vector<Child> child_;
//  std::vector<std::string> bodyName;
//  std::vector<SparseJacobian> contactJaco_;
//  std::vector<SparseJacobian> externalForceJaco_;
//  std::vector<rai_sim::Vec<3>> externalForces_;

#ifndef RAI_BUILD_AS_ONLY
 protected:
//  rai_sim::CollisionSet collisionBodies;
  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape>> visColObj;
  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape, rai_sim::Vec<4>>> visObj;
  std::vector<std::pair<std::string, rai_sim::Vec<4>>> visProps_;
  std::vector<std::pair<std::string, rai_sim::Vec<4>>> visColProps_;
#endif

 private:
  int nbody, dof = -1, jointStateDim = 0, baseJointStateDimMinusOne = 0, baseDOFminusOne = 0;

};

} // object
} // bullet_sim

#endif //BULLETSIM_ARTICULATEDSYSTEM_HPP
