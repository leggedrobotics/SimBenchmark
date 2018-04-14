//
// Created by kangd on 14.04.18.
//

#include "OdeCylinder.hpp"

namespace ode_sim {
namespace object {

OdeCylinder::OdeCylinder(double radius,
                         double height,
                         double mass,
                         dWorldID worldID,
                         dSpaceID spaceID,
                         benchmark::CollisionGroupType collisionGroup,
                         benchmark::CollisionGroupType collisionMask)
    : OdeSingleBodyObject(worldID, spaceID) {

  // body
  body_ = dBodyCreate(worldID);

  // geometry
  geometry_ = dCreateCylinder(spaceID, radius, height);
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
  dMassSetCylinderTotal(&mass_, mass, 3, radius, height);
  dBodySetMass(body_, &mass_);

  // gyroscopic effect
  dBodySetGyroscopicMode(body_, true);

}
} // object
} // ode_sim