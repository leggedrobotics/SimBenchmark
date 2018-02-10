//
// Created by kangd on 10.02.18.
//

#ifndef BENCHMARK_WORLD_HPP
#define BENCHMARK_WORLD_HPP

#include "World.hpp"
#include <raiGraphics/RAI_graphics.hpp>
#include <configure.hpp>

namespace bullet_sim {

enum VisualizerOption {
  NO_BACKGROUND = 1<<(1),
  DISABLE_INTERACTION = 1<<(2)
};

enum CheckerBoardOption {
  GRID = 1<<(1),
};

class World_RG {

 public:

  /* constructor for visualization */
  World_RG(int windowWidth,
           int windowHeight,
           float cms,
           int flags = 0);

  /* constructor for no visualization */
  World_RG() = default;
  virtual ~World_RG();

  /////////////////////////////////////
  /// Visualization related methods ///
  /////////////////////////////////////
  void loop(double dt, double realTimeFactor = 1.0);
  void visStart();
  void visEnd();

  //////////////////////////////////
  /// adding or removing objects ///
  //////////////////////////////////
  // TODO return ptr
  void addBox(double xLength, double yLength, double zLength, double mass,
              CollisionGroupType collisionGroup = 1, CollisionGroupType collisionMask=-1);
  void addCheckerboard(double gridSize, double xLength, double yLength, double reflectanceI,
                       CollisionGroupType collisionGroup = 1, CollisionGroupType collisionMask=-1, int flags = 0);


 private:
  World world_;

  int visualizerFlags_ = 0;
//  StopWatch watch_, visualizerWatch_; // TODO

  std::unique_ptr<rai_graphics::RAI_graphics> gui_;
  std::unique_ptr<rai_graphics::object::Arrow> contactNormalArrow_;
  std::unique_ptr<rai_graphics::object::Sphere> contactPointMarker_;
  std::unique_ptr<rai_graphics::object::Background> background_;
  std::unique_ptr<rai_graphics::object::Sphere> graphicalComMarker_;
  std::unique_ptr<rai_graphics::object::Arrow> frameX_, frameY_, frameZ_;

  rai_graphics::CameraProp cameraProperty_;
  rai_graphics::LightProp lightProperty_;

  const int windowWidth_ = 800;
  const int windowHeight_ = 600;

  bool isReady_=false;
  bool isEnded_=false;

};

} // bullet_sim

#endif //BENCHMARK_WORLD_HPP
