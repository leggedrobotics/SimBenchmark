//
// Created by kangd on 17.04.18.
//

#ifndef BENCHMARK_ODELINKJOINT_HPP
#define BENCHMARK_ODELINKJOINT_HPP

#include <ode/ode.h>
#include "common/math.hpp"

namespace bo = benchmark::object;
namespace ode_sim {
namespace object {

// world orientation, world pos, parent id, shape, color
typedef std::tuple<benchmark::Mat<3, 3>,
                   benchmark::Vec<3>,
                   int,
                   bo::Shape,
                   benchmark::Vec<4>> VisualObjectData;
// mesh path, size
typedef std::pair<std::string,
                  benchmark::Vec<4>> VisualObjectProperty;
// world orientation, world pos, parent id, shape
typedef std::tuple<benchmark::Mat<3, 3>,
                   benchmark::Vec<3>,
                   int,
                   bo::Shape> AlternativeVisualObjectData;


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
    pos_[0] = *(p.begin());
    pos_[1] = *(p.begin() + 1);
    pos_[2] = *(p.begin() + 2);
  }

  benchmark::Vec<3> axis_;
  benchmark::Vec<3> pos_;
  benchmark::Mat<3,3> rotmat_;
  Type type;

  // ode
  dJointID odeJoint_ = 0;
};


struct LinkInertial {
  LinkInertial() {
    mass_ = 0;
    memset(inertia_.ptr(), 0, 9 * sizeof(double));
    memset(pos_.ptr(), 0, 3 * sizeof(double));
    memset(rotmat_.ptr(), 0, 9 * sizeof(double));
    rotmat_[0] = 1;
    rotmat_[4] = 1;
    rotmat_[8] = 1;
  }
  benchmark::Mat<3, 3> inertia_;
  benchmark::Mat<3, 3> rotmat_;   // w.r.t joint
  benchmark::Vec<3> pos_;         // w.r.t joint
  double mass_;
  dMass odeMass_;
};


struct LinkVisual {
  void visOrientation(const benchmark::Vec<3> &rpy) {
    visObjRotMat_.emplace_back();
    benchmark::rpyToRotMat_intrinsic(rpy, visObjRotMat_.back());
  }

  std::vector<bo::Shape> visshape_;
  std::vector<benchmark::Vec<4>> visShapeParam_;
  std::vector<benchmark::Vec<3>> visObjOrigin_;     // w.r.t joint
  std::vector<benchmark::Mat<3, 3>> visObjRotMat_;  // w.r.t joint
  std::vector<benchmark::Vec<4>> visColor_;
  std::vector<std::string> meshFileNames_;
};


struct LinkCollision {
  void colOrientation(const benchmark::Vec<3> &rpy) {
    colObjRotMat_.emplace_back();
    benchmark::rpyToRotMat_intrinsic(rpy, colObjRotMat_.back());
  }

  std::vector<bo::Shape> colShape_;
  std::vector<benchmark::Vec<4>> colShapeParam_;
  std::vector<benchmark::Vec<3>> colObjOrigin_;     // w.r.t joint
  std::vector<benchmark::Mat<3, 3>> colObjRotMat_;  // w.r.t joint
  std::vector<dGeomID> odeGeometries_;
};


struct Link {

  std::vector<Link> childrenLinks_;

  std::vector<Link> fixedLinks_;

  LinkInertial inertial_;

  LinkCollision collision_;

  LinkVisual visual_;

  Joint parentJoint_;

  std::string name_;
  std::string parentName_;
  std::string parentJointName_;

  int bodyIdx_;
  int parentIdx_;

  dBodyID odeBody_ = 0;

  /// collision ode
//  std::vector<MetrialProp *> matrialProps_;
//  void initCollisions(std::vector<AlternativeVisualObjectData> &collect,
//                      std::vector<VisualObjectProperty> &props);

  int numberOfBodiesFromHere() {
    int nbody = 1;
    for (auto &ch : childrenLinks_)
      nbody += ch.numberOfBodiesFromHere();
    return nbody;
  }
};

}
}

#endif //BENCHMARK_ODELINKJOINT_HPP
