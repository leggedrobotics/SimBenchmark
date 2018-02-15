//
// Created by kangd on 11.02.18.
//

#include "odeSim/object/Box.hpp"

ode_sim::object::Box::Box(double xlength,
                          double ylength,
                          double zlength,
                          double mass,
                          dWorldID worldID,
                          dSpaceID spaceID)
    : SingleBodyObject(worldID, spaceID) {

  // body
  body_ = dBodyCreate(worldID);

  // geometry
  geometry_ = dCreateBox(spaceID, xlength, ylength, zlength);
  dGeomSetBody(geometry_, body_);

  // position and orientation
  dMatrix3 R;
  dRSetIdentity(R);
  dBodySetPosition(body_, 0, 0, 0);
  dBodySetRotation(body_, R);

  // mass and local inertia
  dMassSetBoxTotal(&mass_, mass, xlength, ylength, zlength);

  // gyroscopic effect
  dBodySetGyroscopicMode(body_, true);
}

ode_sim::object::Box::~Box() {
  dBodyDestroy(body_);
  dGeomDestroy(geometry_);
}
