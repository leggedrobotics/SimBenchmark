//
// Created by kangd on 19.02.18.
//

#include "WorldRG.hpp"

namespace benchmark {

WorldRG::WorldRG(int windowWidth, int windowHeight, float cms, int flags) :
    windowWidth_(windowWidth),
    windowHeight_(windowHeight),
    visualizerFlags_(flags) {

  gui_.reset(new rai_graphics::RAI_graphics(windowWidth, windowHeight));
  graphicalComMarker_.reset(new rai_graphics::object::Sphere(0.05, false));
  contactNormalArrow_.reset(new rai_graphics::object::Arrow(0.04 * cms, 0.08 * cms, 2 * cms, 0.5 * cms));
  contactPointMarker_.reset(new rai_graphics::object::Sphere(0.1 * cms));
  frameX_.reset(new rai_graphics::object::Arrow(0.02 * cms, 0.04 * cms, 0.5 * cms, 0.08 * cms));
  frameY_.reset(new rai_graphics::object::Arrow(0.02 * cms, 0.04 * cms, 0.5 * cms, 0.08 * cms));
  frameZ_.reset(new rai_graphics::object::Arrow(0.02 * cms, 0.04 * cms, 0.5 * cms, 0.08 * cms));

  watch_.start();
  visualizerWatch_.start();
  lightProperty_.pos_light = {0.0, 10.0, 10.0};

  if (flags & NO_BACKGROUND) {
  } else {
    background_.reset(new rai_graphics::object::Background("sky"));
    gui_->addBackground(background_.get());
  }

  gui_->setCameraProp(cameraProperty_);
  gui_->setLightProp(lightProperty_);

  Eigen::Vector3d temp(0, 0, -1);
  contactNormalArrow_->setPos(temp);
  contactPointMarker_->setPos(temp);
  contactNormalArrow_->setColor({1.0, 0.0, 0.0});
  contactPointMarker_->setColor({1.0, 0.0, 0.0});
  gui_->addObject(contactNormalArrow_.get());
  gui_->addObject(contactPointMarker_.get());
  gui_->addObject(graphicalComMarker_.get());

  gui_->addObject(frameX_.get());
  gui_->addObject(frameY_.get());
  gui_->addObject(frameZ_.get());

  contactNormalArrow_->setVisibility(false);
  contactPointMarker_->setVisibility(false);
  graphicalComMarker_->setVisibility(false);
  graphicalComMarker_->setColor({0.1, 0.1, 0.1});

  frameX_->setVisibility(false);
  frameY_->setVisibility(false);
  frameZ_->setVisibility(false);
  gui_->changeMenuFontSize(1, 2);
}

WorldRG::~WorldRG() {
  if(!isEnded_ && isReady_)
    visEnd();
}

void WorldRG::loop(double dt, double realTimeFactor) {
  while (visualizerLoop(dt, realTimeFactor))
    integrate(dt);
}

void WorldRG::visStart() {
  RAIFATAL_IF(!gui_, "use different constructor for visualization");
  RAIFATAL_IF(gui_->isReady(), "call vis start only once. It is automatically called in loop");
  gui_->start();
  RAIINFO("Waiting for visualizer");
  while (!gui_->isReady())
    usleep(5000);
  RAIINFO("Simulation starting");
  isReady_ = true;
}

void WorldRG::visEnd() {
  gui_->end();
//  for (auto& obj: asHandles_) {
//    for (auto go: obj.visual())
//      delete go;
//    for (auto go: obj.alternateVisual())
//      delete go;
//  }

  for (auto& obj: sbHandles_) {
    for (auto go: obj.visual())
      delete go;
    for (auto go: obj.alternateVisual())
      delete go;
  }

  isEnded_ = true;
}

bool WorldRG::visualizerLoop(double dt, double realTimeFactor) {
  if (!isReady_)
    visStart();
  double timeLaps = watch_.measure();
  usleep(std::max(0.0, (dt / gui_->getRealTimeFactor() / realTimeFactor - timeLaps) * 1e6));
  watch_.start();

  if(visualizerWatch_.measure() > 1.0/80.0) {
    updateFrame();
    visualizerWatch_.start();
  }

//  TODO?
//  if(objToInteract_)
//    objToInteract_->setExternalForce(interactionForce_, objToInteractLocalIdx_);

  //// toggle (0) is for pausing simulation
  RAIINFO_IF(gui_->getCustomToggleState(10), "Simulation Paused")
  bool paused = false;
  while (gui_->getCustomToggleState(10)) {
    usleep(10000);
    paused = true;
  }
  RAIINFO_IF(paused, "Simulation Resumed")

  if (gui_->isQuitting()) {
    visEnd();
    return false;
  } else
    return true;
}

void WorldRG::cameraFollowObject(rai_graphics::object::SingleBodyObject *followingObject,
                                  Eigen::Vector3d relativePosition) {
  RAIFATAL_IF(!gui_, "visualizer is not running")
  RAIFATAL_IF(!followingObject, "cameraFollowObject error: no graphical object")
  cameraProperty_.toFollow = followingObject;
  cameraProperty_.relativeDist = relativePosition;
  gui_->setCameraProp(cameraProperty_);
}

void WorldRG::cameraFollowObject(SingleBodyHandle followingObject,
                                  Eigen::Vector3d relativePosition) {
  RAIFATAL_IF(!gui_, "visualizer is not running")
  cameraFollowObject(followingObject.visual()[0], relativePosition);
}

void WorldRG::setLightPosition(float x, float y, float z) {
  RAIFATAL_IF(!gui_, "RaiSim is running without visualization")
  lightProperty_.pos_light = {x, y, z};
  gui_->setLightProp(lightProperty_);
}

void WorldRG::checkFileExistance(std::string nm) {
  std::ifstream model_file(nm);
  RAIFATAL_IF(!model_file, "Error opening file: " << nm);
}
void WorldRG::processSingleBody(SingleBodyHandle handle) {
  sbHandles_.push_back(handle);

  for (auto *go: handle.visual())
    processGraphicalObject(go, 0);

  for (auto *av: handle.alternateVisual())
    processGraphicalObject(av, 0);

  if(gui_) framesAndCOMobj_.push_back(handle.s_);
}
void WorldRG::processGraphicalObject(rai_graphics::object::SingleBodyObject *go, int li) {
  gui_->addObject(go);
  // TODO?
//  interactionIdx_.insert(std::pair<int, std::pair<int, int>>(go->getSelectableObIndex(),
//                                                             std::pair<int, int>(world_.getObjList().size() - 1, li)));
}
void WorldRG::adjustTransparency(rai_graphics::object::SingleBodyObject *ob, bool hidable) {
  if (!hidable) return;
  if (gui_->getCustomToggleState(1) || gui_->getCustomToggleState(2) || gui_->getCustomToggleState(4))
    ob->setTransparency(0.5);
  else
    ob->setTransparency(1.0);
}

void WorldRG::startRecordingVideo(std::string dir, std::string fileName) {
  gui_->savingSnapshots(dir, fileName);
}

void WorldRG::stopRecordingVideo() {
  gui_->images2Video();
}

} // benchmark