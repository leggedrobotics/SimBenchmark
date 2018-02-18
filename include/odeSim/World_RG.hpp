//
// Created by kangd on 10.02.18.
//

#ifndef ODESIM_WORLD_RG_HPP
#define ODESIM_WORLD_RG_HPP

#define RAI_COLLISION(x) (1<<(x))

#include <raiGraphics/RAI_graphics.hpp>
#include <raiSim/math.hpp>

#include "odeSim/UserHandle.hpp"
#include "Configure.hpp"
#include "World.hpp"

namespace ode_sim {

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
           int flags = 0,
           SolverOption solverOption = SOLVER_STANDARD);

  /* constructor for no visualization */
  World_RG(SolverOption solverOption = SOLVER_STANDARD);
  virtual ~World_RG();

  /////////////////////////////////////
  /// Visualization related methods ///
  /////////////////////////////////////
  void loop(double dt, double realTimeFactor = 1.0);
  void visStart();
  void visEnd();
  void cameraFollowObject(SingleBodyHandle followingObject, Eigen::Vector3d relativePosition);
  void cameraFollowObject(rai_graphics::object::SingleBodyObject *followingObject, Eigen::Vector3d relativePosition);
  void setLightPosition(float x, float y, float z);
  bool visualizerLoop(double dt, double realTimeFactor = 1.0);
  void updateFrame();

  //////////////////////////////////
  /// adding or removing objects ///
  //////////World////////////////////////
  SingleBodyHandle addSphere(double radius, double mass,
                             CollisionGroupType collisionGroup = 1, CollisionGroupType collisionMask=-1);
  SingleBodyHandle addBox(double xLength, double yLength, double zLength, double mass,
                          CollisionGroupType collisionGroup = 1, CollisionGroupType collisionMask = -1);
  SingleBodyHandle addCheckerboard(double gridSize, double xLength, double yLength, double reflectanceI,
                                   CollisionGroupType collisionGroup = 1, CollisionGroupType collisionMask = -1,
                                   int flags = 0);
  SingleBodyHandle addCapsule(double radius, double height, double mass,
                              CollisionGroupType collisionGroup = 1, CollisionGroupType collisionMask=-1);

  //////////////////////////
  /// simulation methods ///
  //////////////////////////
  void integrate(double dt);
  void setGravity(Eigen::Vector3d gravity);
  void setERP(double erp);

 private:
  void processSingleBody(SingleBodyHandle handle);
  void processGraphicalObject(rai_graphics::object::SingleBodyObject* go, int li);
  void adjustTransparency(rai_graphics::object::SingleBodyObject* ob, bool hidable);

  World world_;
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

} // odesim_sim

#endif //ODESIM_WORLD_RG_HPP
