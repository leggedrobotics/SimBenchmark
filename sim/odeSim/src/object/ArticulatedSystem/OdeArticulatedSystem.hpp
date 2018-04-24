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

  void updateVisuals();

  const EigenVec getGeneralizedCoordinate() override;
  const EigenVec getGeneralizedVelocity() override;
  const EigenVec getGeneralizedForce() override;
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) override;

  int getDOF() override;
  int getStateDimension() override;

  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) override;
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) override;
  void setGeneralizedCoordinate(std::initializer_list<double> jointState) override;
  void setGeneralizedVelocity(std::initializer_list<double> jointVel) override;
  void setGeneralizedForce(std::initializer_list<double> tau) override;
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) override;
  void setGeneralizedForce(const Eigen::VectorXd &tau) override;
  void setColor(Eigen::Vector4d color) override;

  const std::vector<Joint *> &getJoints() const;
  const std::vector<Link *> &getLinks() const;

 private:
  void init();

  /**
   * initialize body index, parent index and make link vector (links_)
   *
   * @param link
   */
  void initIdx(Link &link);

  /**
   * initialize visual objects (position, orientation, shape, size, color)
   * collect, props are output of function
   * @param link
   * @param parentRot_w
   * @param parentPos_w
   * @param collect
   * @param props
   */
  void initVisuals(Link &link,
                   benchmark::Mat<3, 3> &parentRot_w,
                   benchmark::Vec<3> &parentPos_w,
                   std::vector<VisualObjectData> &collect,
                   std::vector<VisualObjectProperty> &props);

  /**
   * initialize ODE collision objects
   *
   * @param link
   * @param parentRot_w
   * @param parentPos_w
   * @param collect
   * @param props
   */
  void initCollisions(Link &link,
                        benchmark::Mat<3, 3> &parentRot_w,
                        benchmark::Vec<3> &parentPos_w,
                        std::vector<AlternativeVisualObjectData> &collect,
                        std::vector<VisualObjectProperty> &props);

  void initInertials(Link &link);

  /**
   * initialize children joint of link includeing ODE joint objects
   *
   * @param link
   * @param parentRot_w
   * @param parentPos_w
   */
  void initJoints(Link &link, benchmark::Mat<3, 3> &parentRot_w, benchmark::Vec<3> &parentPos_w);

  void processLinkFromUrdf(boost::shared_ptr<const urdf::Link> urdfLink,
                             Link &raiLink,
                             std::vector<std::string> &jointsOrder);

  /**
   * update joint position recursively from generalized coordinate
   *
   * @param link
   * @param parentRot_w
   * @param parentPos_w
   */
  void updateJointPos(Link &link,
                      benchmark::Mat<3, 3> &parentRot_w,
                      benchmark::Vec<3> &parentPos_w);

  /**
   * update joint velocity recursively from generalized velocity
   *
   * @param link
   * @param parentRot_w
   * @param parentPos_w
   */
//  void updateJointVelocity(Link &link,
//                           benchmark::Mat<3, 3> &parentRot_w,
//                           benchmark::Vec<3> &parentPos_w);

  std::vector<std::string> jointsNames_;

  // the head of links_ is the pointer of rootLink
  std::vector<Link *> links_;
  std::vector<Joint *> joints_;

  Link rootLink_;
  Joint rootJoint_;

  dWorldID worldID_ = 0;
  dSpaceID spaceID_ = 0;

};

} // object
} // ode_sim

#endif //BENCHMARK_ODEARTICULATEDSYSTEM_HPP
