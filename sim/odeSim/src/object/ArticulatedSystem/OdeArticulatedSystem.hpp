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
#include "OdeLinkJoint.hpp"

namespace bo = benchmark::object;

namespace ode_sim {
namespace object {

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

  // recursively initialize link data
  void initVisuals(Link &link,
                   benchmark::Mat<3, 3> &parentRot_w,
                   benchmark::Vec<3> &parentPos_w,
                   std::vector<VisualObjectData> &collect,
                   std::vector<VisualObjectProperty> &props);
  void initCollisions(Link &link,
                        benchmark::Mat<3, 3> &parentRot_w,
                        benchmark::Vec<3> &parentPos_w,
                        std::vector<AlternativeVisualObjectData> &collect,
                        std::vector<VisualObjectProperty> &props);
  void initInertial(Link &link);

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
