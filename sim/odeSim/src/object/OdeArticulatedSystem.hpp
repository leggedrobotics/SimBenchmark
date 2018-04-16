//
// Created by kangd on 15.04.18.
//

#ifndef BENCHMARK_ODEARTICULATEDSYSTEM_HPP
#define BENCHMARK_ODEARTICULATEDSYSTEM_HPP

#include <ode/ode.h>
#include <string>
#include <iostream>
#include <urdf_parser/urdf_parser.h>

#include "common/interface/ArticulatedSystemInterface.hpp"

#include "object/OdeObject.hpp"

namespace bo = benchmark::object;

namespace ode_sim {
namespace object {

/// joint struct
struct Joint {
  enum Type {
    FIXED,
    REVOLUTE,
    PRISMATIC,
    FLOATING
  };

  void jointAxis(std::initializer_list<double> a) {
    axis_[0] = *(a.begin());
    axis_[1] = *(a.begin() + 1);
    axis_[2] = *(a.begin() + 2);
  }

  void jointPosition(std::initializer_list<double> p) {
    pos_P_[0] = *(p.begin());
    pos_P_[1] = *(p.begin() + 1);
    pos_P_[2] = *(p.begin() + 2);
  }

  benchmark::Vec<3> axis_;
  benchmark::Vec<3> pos_P_;
  benchmark::Vec<3> RPY;
  Type type;

  // ode
  dJointID odeJoint_ = 0;
};

/// link struct
struct Link {
  int bodyIdx;
  int parentIdx;

  /// basic props
  Joint joint;
  bool registered = false;
  std::string name, parentName;
  std::string parentJointName;

  /// collision (ode)
  dBodyID odeBody_ = 0;
  dMass odeMass_;
  std::vector<dGeomID> odeGeometries_;

  /// visual props
  std::vector<benchmark::object::Shape> visshape;
  std::vector<benchmark::Vec<4>> visShapeParam;
  std::vector<benchmark::Vec<3>> visObjOrigin;
  std::vector<benchmark::Mat<3, 3>> visObjRotMat;
  std::vector<benchmark::Vec<4>> visColor;
  std::vector<std::string> meshFileNames;
  std::vector<benchmark::Vec<4>> calShapeParam;

  /// children
  std::vector<Link> childrenLinks;
  std::vector<Link> fixedBodies;

  void initVisuals(std::vector<std::tuple<benchmark::Mat<3, 3>,
                                          benchmark::Vec<3>,
                                          int,
                                          bo::Shape,
                                          benchmark::Vec<4>>> &collect,
                   std::vector<std::pair<std::string, benchmark::Vec<4>>> &props);

  int numberOfBodiesFromHere() {
    int nbody = 1;
    for (auto &ch : childrenLinks)
      nbody += ch.numberOfBodiesFromHere();
    return nbody;
  }

  int jointIdx(std::string &nm, std::vector<std::string> &jointsNames) {
    for (uint i = 0; i < jointsNames.size(); i++)
      if (nm == jointsNames[i]) return int(i);
    return -1;
  }

  void visOrientation(const benchmark::Vec<3> &rpy) {
    visObjRotMat.emplace_back();
    benchmark::rpyToRotMat_intrinsic(rpy, visObjRotMat.back());
  }
};

/// ArticulatedSystem class
class OdeArticulatedSystem: public bo::ArticulatedSystemInterface,
                            public object::OdeObject {

 public:
  OdeArticulatedSystem(std::string urdfFile,
                       const dWorldID worldID,
                       const dSpaceID spaceID);
  virtual ~OdeArticulatedSystem();

  const EigenVec getGeneralizedCoordinate() override;
  const EigenVec getGeneralizedVelocity() override;
  const EigenVec getGeneralizedForce() override;
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) override;
  int getDOF() override;

  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) override;
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) override;
  void setGeneralizedCoordinate(std::initializer_list<double> jointState) override;
  void setGeneralizedVelocity(std::initializer_list<double> jointVel) override;
  void setGeneralizedForce(std::initializer_list<double> tau) override;
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) override;
  void setGeneralizedForce(const Eigen::VectorXd &tau) override;
  void setColor(Eigen::Vector4d color) override;

 private:
  void init();
  void processLinkFromUrdf(boost::shared_ptr<const urdf::Link> urdfLink,
                           Link &raiLink,
                           std::vector<std::string> &jointsOrder);

  std::vector<std::string> jointsNames_;
  std::vector<Link> links_;

  dWorldID worldID_ = 0;
  dSpaceID spaceID_ = 0;

};

} // object
} // ode_sim

#endif //BENCHMARK_ODEARTICULATEDSYSTEM_HPP
