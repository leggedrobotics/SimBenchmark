//
// Created by kangd on 13.02.18.
//

#include "OdeSphere.hpp"

ode_sim::object::OdeSphere::OdeSphere(double radius,
                                      double mass,
                                      dWorldID worldID,
                                      dSpaceID spaceID,
                                      benchmark::CollisionGroupType collisionGroup,
                                      benchmark::CollisionGroupType collisionMask)
    : OdeSingleBodyObject(worldID, spaceID) {

  // body
  body_ = dBodyCreate(worldID);

  // geometry
  geometry_ = dCreateSphere(spaceID, radius);
  dGeomSetBody(geometry_, body_);

  // material prop
  dGeomSetData(geometry_, &matrialProp_);

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
