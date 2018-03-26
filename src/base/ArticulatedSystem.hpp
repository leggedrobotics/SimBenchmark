//
// Created by kangd on 25.03.18.
//

#ifndef BENCHMARK_ARTICULATEDSYSTEM_HPP
#define BENCHMARK_ARTICULATEDSYSTEM_HPP

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

class ArticulatedSystem {

 public:
  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape, rai_sim::Vec<4>>>& getVisOb() {
    return visObj;
  };

  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape>>& getVisColOb() {
    return visColObj;
  };

 public:
  // orientation, position, link_id, shape, color
  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape>> visColObj;
  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape, rai_sim::Vec<4>>> visObj;
  std::vector<std::pair<std::string, rai_sim::Vec<4>>> visProps_;
  std::vector<std::pair<std::string, rai_sim::Vec<4>>> visColProps_;

};

} // object
} // benchmark

#endif //BENCHMARK_ARTICULATEDSYSTEM_HPP
