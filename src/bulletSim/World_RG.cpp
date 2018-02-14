//
// Created by kangd on 10.02.18.
//

#include "bulletSim/World_RG.hpp"

namespace bullet_sim {

World_RG::World_RG(int windowWidth, int windowHeight, float cms, int flags) :
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

World_RG::~World_RG() {
  if(!isEnded_ && isReady_)
    visEnd();
}

SingleBodyHandle World_RG::addBox(double xLength,
                                  double yLength,
                                  double zLength,
                                  double mass,
                                  CollisionGroupType collisionGroup,
                                  CollisionGroupType collisionMask) {

  SingleBodyHandle handle(world_.addBox(xLength, yLength, zLength, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Box(xLength, yLength, zLength, true));
  processSingleBody(handle);
  return handle;
}

SingleBodyHandle World_RG::addSphere(double radius,
                                     double mass,
                                     CollisionGroupType collisionGroup,
                                     CollisionGroupType collisionMask) {
  SingleBodyHandle handle(world_.addSphere(radius, mass, collisionGroup, collisionMask), {}, {});
  if(gui_) handle.visual().push_back(new rai_graphics::object::Sphere(radius, true));
  processSingleBody(handle);
  return handle;
}

SingleBodyHandle World_RG::addCheckerboard(double gridSize,
                                           double xLength,
                                           double yLength,
                                           double reflectanceI,
                                           CollisionGroupType collisionGroup,
                                           CollisionGroupType collisionMask,
                                           int flags) {
  SingleBodyHandle handle(world_.addCheckerboard(gridSize, xLength, yLength, reflectanceI, collisionGroup, collisionMask), {}, {});
  handle.hidable = false;
  if(gui_) {
    handle.visual().push_back(new rai_graphics::object::CheckerBoard(gridSize, xLength, yLength, reflectanceI));
    static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0])->gridMode = flags & GRID;
    gui_->addCheckerBoard(static_cast<rai_graphics::object::CheckerBoard *>(handle.visual()[0]));
  }
  sbHandles_.push_back(handle);
  return handle;
}

ArticulatedSystemHandle World_RG::addArticulatedSystem(std::string nm,
                                                       CollisionGroupType collisionGroup,
                                                       CollisionGroupType collisionMask) {
  ArticulatedSystemHandle handle(world_.addArticulatedSystem(nm, collisionGroup, collisionMask), {}, {});
  if(!gui_) {
    asHandles_.push_back(handle);
    return handle;
  }
//
//  for (int i = 0; i < handle->visObj.size(); i++) {
//    switch (std::get<3>(handle->visObj[i])) {
//      case object::Shape::Box:
//        handle.visual().push_back(new rai_graphics::object::Box(handle->visProps_[i].second.v[0],
//                                                                handle->visProps_[i].second.v[1],
//                                                                handle->visProps_[i].second.v[2], true));
//        break;
//      case object::Shape::Cylinder:
//        handle.visual().push_back(new rai_graphics::object::Cylinder(handle->visProps_[i].second.v[0],
//                                                                     handle->visProps_[i].second.v[1], true));
//        break;
//      case object::Shape::Sphere:
//        handle.visual().push_back(new rai_graphics::object::Sphere(handle->visProps_[i].second.v[0], true));
//        break;
//      case object::Shape::Mesh:
//        checkFileExistance(nm + handle->visProps_[i].first);
//        handle.visual().push_back(new rai_graphics::object::Mesh(nm + handle->visProps_[i].first,
//                                                                 handle->visProps_[i].second.v[0]));
//        break;
//    }
//    handle.visual().back()->setColor({float(std::get<4>(handle->visObj[i]).v[0]),
//                                      float(std::get<4>(handle->visObj[i]).v[1]),
//                                      float(std::get<4>(handle->visObj[i]).v[2])});
//    processGraphicalObject(handle.visual().back(), std::get<2>(handle->visObj[i]));
//  }
//
//  for (int i = 0; i < handle->visColObj.size(); i++) {
//    switch (std::get<3>(handle->visColObj[i])) {
//      case object::Shape::Box:
//        handle.alternateVisual().push_back(new rai_graphics::object::Box(handle->visColProps_[i].second.v[0],
//                                                                         handle->visColProps_[i].second.v[1],
//                                                                         handle->visColProps_[i].second.v[2],
//                                                                         true));
//        break;
//      case object::Shape::Cylinder:
//        handle.alternateVisual().push_back(new rai_graphics::object::Cylinder(handle->visColProps_[i].second.v[0],
//                                                                              handle->visColProps_[i].second.v[1],
//                                                                              true));
//        break;
//      case object::Shape::Sphere:
//        handle.alternateVisual().push_back(new rai_graphics::object::Sphere(handle->visColProps_[i].second.v[0],
//                                                                            true));
//        break;
//      case object::Shape::Mesh:
//      RAIFATAL("mesh collision body is not supported yet");
//        break;
//      default: RAIFATAL("unsupported type: ");
//        break;
//    }
//    processGraphicalObject(handle.alternateVisual().back(), std::get<2>(handle->visColObj[i]));
//  }

  asHandles_.push_back(handle);
  return handle;
}

void World_RG::loop(double dt, double realTimeFactor) {
  while (visualizerLoop(dt, realTimeFactor))
    integrate(dt);
}

void World_RG::integrate(double dt) {
  world_.integrate(dt);
}

void World_RG::visStart() {
  RAIFATAL_IF(!gui_, "use different constructor for visualization");
  RAIFATAL_IF(gui_->isReady(), "call vis start only once. It is automatically called in loop");
  gui_->start();
  RAIINFO("Waiting for visualizer");
  while (!gui_->isReady())
    usleep(5000);
  RAIINFO("Simulation starting");
  isReady_ = true;
}

void World_RG::visEnd() {
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

//  for (auto& obj: comHandles_) {
//    for (auto go: obj.visual())
//      delete go;
//    for (auto go: obj.alternateVisual())
//      delete go;
//  }
  isEnded_ = true;
}

bool World_RG::visualizerLoop(double dt, double realTimeFactor) {
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

void World_RG::cameraFollowObject(rai_graphics::object::SingleBodyObject *followingObject,
                                  Eigen::Vector3d relativePosition) {
  RAIFATAL_IF(!gui_, "visualizer is not running")
  RAIFATAL_IF(!followingObject, "cameraFollowObject error: no graphical object")
  cameraProperty_.toFollow = followingObject;
  cameraProperty_.relativeDist = relativePosition;
  gui_->setCameraProp(cameraProperty_);
}

void World_RG::cameraFollowObject(SingleBodyHandle followingObject,
                                  Eigen::Vector3d relativePosition) {
  cameraFollowObject(followingObject.visual()[0], relativePosition);
}

void World_RG::setLightPosition(float x, float y, float z) {
  RAIFATAL_IF(!gui_, "RaiSim is running without visualization")
  lightProperty_.pos_light = {x, y, z};
  gui_->setLightProp(lightProperty_);
}

void World_RG::updateFrame() {
  RAIFATAL_IF(!gui_, "use different constructor for visualization")
  const bool showAlternateGraphicsIfexists = gui_->getCustomToggleState(3);

//  TODO articulated system
//  for (auto &as : asHandles_) {
//    Vec<4> quat;
//    Vec<3> pos, jointPos_W;
//    Mat<3, 3> rot_WB, rotTemp;
//
//    if (showAlternateGraphicsIfexists) {
//      /// update collision objects
//      for (int i = 0; i < as->getVisColOb().size(); i++) {
//        as.alternateVisual()[i]->setVisibility(true);
//        int parentId = std::get<2>(as->getVisColOb()[i]);
//        as->getBodyPose(parentId, rot_WB, jointPos_W);
//        matvecmul(rot_WB, std::get<1>(as->getVisColOb()[i]), pos);
//        as.alternateVisual()[i]->setPos(jointPos_W.v[0] + pos.v[0],
//                                        jointPos_W.v[1] + pos.v[1],
//                                        jointPos_W.v[2] + pos.v[2]);
//        matmul(rot_WB, std::get<0>(as->getVisColOb()[i]), rotTemp);
//        rotMatToQuat(rotTemp, quat);
//        as.alternateVisual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
//        adjustTransparency(as.alternateVisual()[i], as.hidable);
//      }
//
//      for (int i = 0; i < as->getVisOb().size(); i++)
//        as.visual()[i]->setVisibility(false);
//    } else {
//      for (int i = 0; i < as->getVisOb().size(); i++) {
//        as.visual()[i]->setVisibility(true);
//        if (!as.visual()[i]->isVisible()) continue;
//        int parentId = std::get<2>(as->getVisOb()[i]);
//        as->getBodyPose(parentId, rot_WB, jointPos_W);
//        matvecmul(rot_WB, std::get<1>(as->getVisOb()[i]), pos);
//        as.visual()[i]->setPos(jointPos_W.v[0] + pos.v[0],
//                               jointPos_W.v[1] + pos.v[1],
//                               jointPos_W.v[2] + pos.v[2]);
//        matmul(rot_WB, std::get<0>(as->getVisOb()[i]), rotTemp);
//        rotMatToQuat(rotTemp, quat);
//        as.visual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
//        adjustTransparency(as.visual()[i], as.hidable);
//      }
//      for (int i = 0; i < as->getVisColOb().size(); i++)
//        as.alternateVisual()[i]->setVisibility(false);
//    }
//  }

  rai_sim::Vec<3> bodyPosition;
  rai_sim::Vec<4> quat;

  for (auto sb : sbHandles_) {
    sb->getPosition_W(bodyPosition);
    sb->getQuaternion(quat);

    if (!showAlternateGraphicsIfexists || sb.alternateVisual().size() == 0) {
      for (auto *go: sb.alternateVisual()) go->setVisibility(false);
      for (auto *go: sb.visual()) {
        go->setVisibility(true);
        go->setPos({bodyPosition.v[0], bodyPosition.v[1], bodyPosition.v[2]});
        go->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        adjustTransparency(go, sb.hidable);
      }
    } else {
      for (auto *go: sb.visual()) go->setVisibility(false);
      for (auto *go: sb.visual()) {
        go->setVisibility(true);
        go->setPos({bodyPosition.v[0], bodyPosition.v[1], bodyPosition.v[2]});
        go->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
        adjustTransparency(go, sb.hidable);
      }
    }
  }

//  rai_sim::Mat<3, 3> rot;
//  for (auto sb : comHandles_) {
//    sb->getPosition_W(bodyPosition);
//    sb->getRotationMatrix(rot);
//    auto &trans = sb->getTransformation();
//
//    if (!showAlternateGraphicsIfexists || sb.alternateVisual().size() == 0) {
//      for (auto *go: sb.alternateVisual()) go->setVisibility(false);
//      for (int i = 0; i < sb.visual().size(); i++) {
//        Vec<3> pos;
//        Vec<4> quat;
//        matvecmul(rot, trans[i].pos, pos);
//        sb.visual()[i]->setPos(bodyPosition.v[0] + pos.v[0],
//                               bodyPosition.v[1] + pos.v[1],
//                               bodyPosition.v[2] + pos.v[2]);
//        matmul(rot, trans[i].rot, rot);
//        rotMatToQuat(rot, quat);
//        sb.visual()[i]->setOri(quat.v[0], quat.v[1], quat.v[2], quat.v[3]);
//        adjustTransparency(sb.visual()[i], sb.hidable);
//      }
//    }
//  }

  /// contact points
//  if (gui_->getCustomToggleState(1)) {
//    contactPointMarker_->mutexLock();
//    contactPointMarker_->clearGhost();
//    for (auto &pro: *world_.getCollisionProblem()) {
//      Eigen::Vector3d pos = pro.position_W.e();
//      contactPointMarker_->addGhost(pos);
//    }
//    contactPointMarker_->mutexUnLock();
//  } else
//    contactPointMarker_->clearGhost();

  /// contact forces
//  if (gui_->getCustomToggleState(2)) {
//    double maxForce = 0;
//    for (auto &pro: *world_.getCollisionProblem())
//      maxForce = (maxForce < pro.imp_i.norm()) ? pro.imp_i.norm() : maxForce;
//    contactNormalArrow_->mutexLock();
//    contactNormalArrow_->clearGhost();
//    for (auto &pro: world_.getObjList()) {
//      for (auto &con: pro->getPerObjectContact().getContacts()) {
//        const double norm = con.getImpulse()->norm();
//        Eigen::Vector3d pos(con.getPosition().v[0], con.getPosition().v[1], con.getPosition().v[2]);
//        Vec<3> scaledImp;
//        matTransposevecmul(con.getContactFrame(), *con.getImpulse(), scaledImp);
//        vecScalarMul(1.0 / norm, scaledImp);
//        Eigen::Vector3d dir = scaledImp.e();
//        Eigen::Vector3f color(norm / maxForce, 0.2, 1 - norm / maxForce);
//        Eigen::Vector3f scale(norm / maxForce, 1, 1);
//        contactNormalArrow_->addGhostWithVector(pos, dir, color, scale);
//      }
//    }
//    contactNormalArrow_->mutexUnLock();
//  } else
//    contactNormalArrow_->clearGhost();

  /// frames and COM
  if (gui_->getCustomToggleState(4)) {

    frameX_->mutexLock();
    frameY_->mutexLock();
    frameZ_->mutexLock();
    graphicalComMarker_->mutexLock();

    frameX_->clearGhost();
    frameY_->clearGhost();
    frameZ_->clearGhost();
    graphicalComMarker_->clearGhost();
    Eigen::Vector3f colorR(1, 0, 0), colorG(0, 1, 0), colorB(0, 0, 1);
    Eigen::Vector3d xdir, ydir, zdir;

    for (auto *cf: framesAndCOMobj_) {
      if (!cf->isVisualizeFramesAndCom()) continue;
      Eigen::Vector3d pos = cf->getPosition();
      Eigen::Matrix3d dir = cf->getRotationMatrix();
      Eigen::Vector3f scale(1, 1, 1);

      xdir = dir.col(0);
      ydir = dir.col(1);
      zdir = dir.col(2);

      frameX_->addGhostWithVector(pos, xdir, colorR, scale);
      frameY_->addGhostWithVector(pos, ydir, colorG, scale);
      frameZ_->addGhostWithVector(pos, zdir, colorB, scale);
      graphicalComMarker_->addGhost(pos);
    }
    frameX_->mutexUnLock();
    frameY_->mutexUnLock();
    frameZ_->mutexUnLock();
    graphicalComMarker_->mutexUnLock();
  } else {
    frameX_->clearGhost();
    frameY_->clearGhost();
    frameZ_->clearGhost();
    graphicalComMarker_->clearGhost();
  }

  if(visualizerFlags_ & DISABLE_INTERACTION)
    return;

  /// interaction
//  if (gui_->isInteracting()) {
//    auto indices = interactionIdx_.find(gui_->getInteractingObjectID());
//    interactionForce_ = gui_->getInteractionMagnitude();
//    interactionForce_ *= interactionForce_.norm() * 20.0 * world_.getObjList()[indices->second.first]->getMass(indices->second.second);
//    objToInteract_ = world_.getObjList()[indices->second.first];
//    objToInteractLocalIdx_ = indices->second.second;
//    std::stringstream inStr;
//    inStr << std::setprecision(3) << interactionForce_.norm() << "N";
//    gui_->changeMenuText(1, false, inStr.str());
//    gui_->setMenuPositionNextToCursor(1);
//  } else {
//    gui_->changeMenuText(1, false, "");
//    objToInteract_ = nullptr;
//  }

  /// deletion
//  if (gui_->getKeyboardEvent(rai_graphics::KeyboardEvent::DEL)) {
//    auto indices = interactionIdx_.find(gui_->getInteractingObjectID());
//    if (indices != interactionIdx_.end()) {
//      object::Object *obj = world_.getObjList()[indices->second.first];
//      long int id =
//          std::find_if(sbHandles_.begin(), sbHandles_.end(), [obj](const SingleBodyHandle &a) { return a.s_ == obj; })
//              - sbHandles_.begin();
//      if (id == sbHandles_.size()) {
//        long int id = std::find_if(asHandles_.begin(),
//                                   asHandles_.end(),
//                                   [obj](const ArticulatedSystemHandle &a) { return a.s_ == obj; })
//            - asHandles_.begin();
//        if (!id == asHandles_.size())
//          removeObject(asHandles_[id]);
//      } else {
//        removeObject(sbHandles_[id]);
//      }
//      interactionIdx_.erase(gui_->getInteractingObjectID());
//    }
//  }
}

void World_RG::checkFileExistance(std::string nm) {
  std::ifstream model_file(nm);
  RAIFATAL_IF(!model_file, "Error opening file: " << nm);
}

void World_RG::processSingleBody(bullet_sim::SingleBodyHandle handle) {
  sbHandles_.push_back(handle);

  for (auto *go: handle.visual())
    processGraphicalObject(go, 0);

  for (auto *av: handle.alternateVisual())
    processGraphicalObject(av, 0);

  if(gui_) framesAndCOMobj_.push_back(handle.s_);
}

void World_RG::processGraphicalObject(rai_graphics::object::SingleBodyObject *go, int li) {
  gui_->addObject(go);
  // TODO?
//  interactionIdx_.insert(std::pair<int, std::pair<int, int>>(go->getSelectableObIndex(),
//                                                             std::pair<int, int>(world_.getObjList().size() - 1, li)));
}

void World_RG::adjustTransparency(rai_graphics::object::SingleBodyObject *ob, bool hidable) {
  if (!hidable) return;
  if (gui_->getCustomToggleState(1) || gui_->getCustomToggleState(2) || gui_->getCustomToggleState(4))
    ob->setTransparency(0.5);
  else
    ob->setTransparency(1.0);
}
void World_RG::setGravity(Eigen::Vector3d gravity) {
  world_.setGravity({gravity.x(), gravity.y(), gravity.z()});
}

} // bullet_sim