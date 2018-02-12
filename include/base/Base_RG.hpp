//
// Created by kangd on 11.02.18.
//

#ifndef BENCHMARK_BASE_RG_HPP
#define BENCHMARK_BASE_RG_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include <raiSim/math.hpp>

namespace benchmark {

enum VisualizerOption {
  NO_BACKGROUND = 1<<(1),
  DISABLE_INTERACTION = 1<<(2)
};

enum CheckerBoardOption {
  GRID = 1<<(1),
};

class Base_RG {

 public:
  virtual ~Base_RG();

  /////////////////////////////////////
  /// Visualization related methods ///
  /////////////////////////////////////
  virtual void loop(double dt, double realTimeFactor = 1.0) = 0;
  virtual void visStart() = 0;
  virtual void visEnd() = 0;
  virtual void cameraFollowObject(SingleBodyHandle followingObject, Eigen::Vector3d relativePosition) = 0;
  virtual void cameraFollowObject(rai_graphics::object::SingleBodyObject *followingObject, Eigen::Vector3d relativePosition) = 0;
  virtual void setLightPosition(float x, float y, float z) = 0;
  virtual bool visualizerLoop(double dt, double realTimeFactor = 1.0) = 0;
  virtual void updateFrame() = 0;
  virtual void integrate(double dt) = 0;

  //////////////////////////////////
  /// adding or removing objects ///
  //////////World////////////////////////
  virtual SingleBodyHandle addBox(double xLength, double yLength, double zLength, double mass,
                                  CollisionGroupType collisionGroup = 1, CollisionGroupType collisionMask = -1) = 0;
  virtual SingleBodyHandle addCheckerboard(double gridSize, double xLength, double yLength, double reflectanceI,
                                           CollisionGroupType collisionGroup = 1, CollisionGroupType collisionMask = -1,
                                           int flags = 0) = 0;

 protected:
  
  int visualizerFlags_ = 0;
  StopWatch watch_, visualizerWatch_;

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

  std::vector<SingleBodyHandle> sbHandles_;
//  std::vector<ArticulatedSystemHandle> asHandles_;
  std::vector<object::SingleBodyObject *> framesAndCOMobj_;

  bool isReady_=false;
  bool isEnded_=false;

};

} // benchmark

#endif //BENCHMARK_BASE_RG_HPP
