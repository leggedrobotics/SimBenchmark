//
// Created by kangd on 13.02.18.
//

#include <Configure.hpp>
#include "odeSim/object/Sphere.hpp"
#include "SingleBodyObject.hpp"

ode_sim::object::Sphere::Sphere(double radius,
                                double mass,
                                dWorldID worldID,
                                dSpaceID spaceID,
                                CollisionGroupType collisionGroup,
                                CollisionGroupType collisionMask)
    : SingleBodyObject(worldID, spaceID) {

  // body
  body_ = dBodyCreate(worldID);

  // geometry
  geometry_ = dCreateSphere(spaceID, radius);
  dGeomSetBody(geometry_, body_);

  // collision group
  dGeomSetCategoryBits(geometry_, collisionGroup);
  dGeomSetCollideBits(geometry_, collisionMask);

  // position and orientation
  dMatrix3 R;
  dRSetIdentity(R);
  dBodySetPosition(body_, 0, 0, 0);
  dBodySetRotation(body_, R);

  // mass and local inertia
  dMassSetSphereTotal(&mass_, mass, radius);
  dBodySetMass(body_, &mass_);

  // gyroscopic effect
  dBodySetGyroscopicMode(body_, true);
}
