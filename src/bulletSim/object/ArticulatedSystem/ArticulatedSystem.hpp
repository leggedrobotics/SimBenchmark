//
// Created by kangd on 25.03.18.
//

#ifndef BULLETSIM_ARTICULATEDSYSTEM_HPP
#define BULLETSIM_ARTICULATEDSYSTEM_HPP

#include <string>
#include <btBulletCollisionCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>

#include "URDF/BulletUrdfImporter.h"
#include "URDF/MyMultiBodyCreator.h"
#include "URDF/UrdfParser.h"

#include "base/ArticulatedSystem.hpp"
#include "bulletSim/object/ArticulatedSystem/URDF/URDFToBullet.h"
#include "bulletSim/object/Object.hpp"

namespace bullet_sim {
namespace object {

class ArticulatedSystem: public Object, public benchmark::object::ArticulatedSystem {

 public:
  ArticulatedSystem(std::string urdfFile, btMultiBodyDynamicsWorld *world);
  virtual ~ArticulatedSystem();

  void updateVisuals();

 private:
  void initVisuals();
  void initVisualFromCollider(btMultiBodyLinkCollider *linkCollider, int colliderId, std::string meshfile="");
  void initVisualFromCollisionShape(btCollisionShape *collisionShape,
                                    rai_sim::Mat<3, 3> rotMat,
                                    rai_sim::Vec<3> pos,
                                    int id,
                                    std::string meshfile="");

  btMultiBody *multiBody_;
  BulletURDFImporter *urdfImporter_;
};

} // object
} // rai_sim

#endif //BULLETSIM_ARTICULATEDSYSTEM_HPP
