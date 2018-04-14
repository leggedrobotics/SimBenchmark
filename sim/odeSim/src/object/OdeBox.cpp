//
// Created by kangd on 11.02.18.
//

#include "OdeBox.hpp"

ode_sim::object::OdeBox::OdeBox(double xlength,
                          double ylength,
                          double zlength,
                          double mass,
                          dWorldID worldID,
                          dSpaceID spaceID,
                          benchmark::CollisionGroupType collisionGroup,
                          benchmark::CollisionGroupType collisionMask)
    : OdeSingleBodyObject(worldID, spaceID) {

  // body
  body_ = dBodyCreate(worldID);

  // geometry
  geometry_ = dCreateBox(spaceID, xlength, ylength, zlength);
  dGeomSetBody(geometry_, body_);

  // material prop
  dGeomSetData(geometry_, &matrialProp_);

  // collision group
  dGeomSetCategoryBits(geometry_, collisionGroup);
  dGeomSetCollideBits(geometry_, collisionMask);

  // material prop
  dGeomSetData(geometry_, &matrialProp_);

  // position and orientation
  dMatrix3 R;
  dRSetIdentity(R);
  dBodySetPosition(body_, 0, 0, 0);
  dBodySetRotation(body_, R);

  // mass and local inertia
  dMassSetBoxTotal(&mass_, mass, xlength, ylength, zlength);
  dBodySetMass(body_, &mass_);

  // gyroscopic effect
  dBodySetGyroscopicMode(body_, true);
}

ode_sim::object::OdeBox::~OdeBox() {
}
