//
// Created by kangd on 10.02.18.
//

#include "World_RG.hpp"

bullet_sim::World_RG::World_RG(int windowWidth, int windowHeight, float cms, int flags) :
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

bullet_sim::World_RG::World_RG() {}

bullet_sim::World_RG::~World_RG() {

}
void bullet_sim::World_RG::addBox(double xLength,
                                  double yLength,
                                  double zLength,
                                  double mass,
                                  CollisionGroupType collisionGroup,
                                  CollisionGroupType collisionMask) {

  world_.addBox(xLength, yLength, zLength, mass, collisionGroup, collisionMask);

}
void bullet_sim::World_RG::addCheckerboard(double gridSize,
                                           double xLength,
                                           double yLength,
                                           double reflectanceI,
                                           CollisionGroupType collisionGroup,
                                           CollisionGroupType collisionMask,
                                           int flags) {

}
