//
// Created by kangd on 12.02.18.
//

#include "odeSim/object/CheckerBoard.hpp"

ode_sim::object::CheckerBoard::CheckerBoard(dWorldID worldId) : SingleBodyObject(worldId) {

  // body
  body_ = dBodyCreate(worldId);

  // geometry
  geometry_ = dCreatePlane(0, 0, 0, 1, 0);
  dGeomSetBody(geometry_, body_);

  // position and orientation
  dMatrix3 R;
  dRSetIdentity(R);
  dBodySetPosition(body_, 0, 0, 0);
  dBodySetRotation(body_, R);

  // mass and local inertia
  dMassSetZero(&mass_);

  // gyroscopic effect
  dBodySetGyroscopicMode(body_, false);
}

ode_sim::object::CheckerBoard::~CheckerBoard() {
  dGeomDestroy(geometry_);
}
