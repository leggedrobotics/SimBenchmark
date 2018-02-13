//
// Created by kangd on 13.02.18.
//

#include "Sphere.hpp"

ode_sim::object::Sphere::Sphere(double radius, double mass, dWorldID worldID, dSpaceID spaceID)
    : SingleBodyObject(worldID, spaceID) {

  // body
  body_ = dBodyCreate(worldID);

  // geometry
  geometry_ = dCreateSphere(spaceID, radius);
  dGeomSetBody(geometry_, body_);

  // position and orientation
  dMatrix3 R;
  dRSetIdentity(R);
  dBodySetPosition(body_, 0, 0, 0);
  dBodySetRotation(body_, R);

  // mass and local inertia
  dMassSetSphereTotal(&mass_, mass, radius);

  // gyroscopic effect
  dBodySetGyroscopicMode(body_, true);
}
