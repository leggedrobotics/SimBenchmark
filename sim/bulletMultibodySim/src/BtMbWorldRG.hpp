//
// Created by kangd on 24.05.18.
//

#ifndef BENCHMARK_BTMBWORLDRG_HPP
#define BENCHMARK_BTMBWORLDRG_HPP

#include <raiGraphics/RAI_graphics.hpp>
#include "common/math.hpp"
#include "common/Configure.hpp"
#include "UserHandle.hpp"

namespace bullet_multibody_sim {

class BtMbWorldRG {

 public:

  /* constructor for visualization */
  BtMbWorldRG(int windowWidth,
              int windowHeight,
              float cms,
              int flags = 0);

  /* constructor for no visualization */
  BtMbWorldRG() = default;
  virtual ~BtMbWorldRG();

  /// Visualization related methods
  virtual void loop(double dt, double realTimeFactor = 1.0);
  virtual void visStart();
  virtual void visEnd();
  virtual void cameraFollowObject(SingleBodyHandle followingObject, Eigen::Vector3d relativePosition);
  virtual void cameraFollowObject(rai_graphics::object::SingleBodyObject *followingObject, Eigen::Vector3d relativePosition);
  virtual void setLightPosition(float x, float y, float z);
  virtual bool visualizerLoop(double dt, double realTimeFactor = 1.0);
  virtual void updateFrame();
  virtual void startRecordingVideo(std::string dir, std::string fileName);
  virtual void stopRecordingVideo();

  /// pure virtual getter, setter
  virtual int getNumObject() = 0;
  virtual int getWorldNumContacts() = 0;

  /// pure virtual simulation methods
  virtual void integrate(double dt) = 0;
  virtual void integrate1(double dt) = 0; // velocity updates
  virtual void integrate2(double dt) = 0; // position updates
  virtual void setGravity(Eigen::Vector3d gravity) = 0;
  virtual void setERP(double erp, double erp2, double frictionErp) = 0;

 protected:
  virtual void checkFileExistance(std::string nm);
  virtual void processSingleBody(SingleBodyHandle handle);
  virtual void processGraphicalObject(rai_graphics::object::SingleBodyObject* go, int li);
  virtual void adjustTransparency(rai_graphics::object::SingleBodyObject* ob, bool hidable);

  // object list
  std::vector<SingleBodyHandle> sbHandles_;
  std::vector<object::SingleBodyObjectInterface *> framesAndCOMobj_;

  // gui objects
  std::unique_ptr<rai_graphics::RAI_graphics> gui_;
  std::unique_ptr<rai_graphics::object::Arrow> contactNormalArrow_;
  std::unique_ptr<rai_graphics::object::Sphere> contactPointMarker_;
  std::unique_ptr<rai_graphics::object::Background> background_;
  std::unique_ptr<rai_graphics::object::Sphere> graphicalComMarker_;
  std::unique_ptr<rai_graphics::object::Arrow> frameX_, frameY_, frameZ_;

  // gui properties
  rai_graphics::CameraProp cameraProperty_;
  rai_graphics::LightProp lightProperty_;

  // gui watch timer
  StopWatch watch_, visualizerWatch_;

  // gui window size
  const int windowWidth_ = 800;
  const int windowHeight_ = 600;

  // gui option
  int visualizerFlags_ = 0;

  bool isReady_=false;
  bool isEnded_=false;
};

} // bullet_multibody_sim


#endif //BENCHMARK_BTMBWORLDRG_HPP
