//
// Created by kangd on 25.03.18.
//

#include <c++/5/vector>
#include <raiSim/math.hpp>
#include "ArticulatedSystem.hpp"

namespace bullet_sim {
namespace object {

ArticulatedSystem::ArticulatedSystem(std::string urdfFile, btMultiBodyDynamicsWorld *world) {

  BulletURDFImporter importer(0, 0, 1.0, 0);
  bool loadOK = importer.loadURDF(urdfFile.c_str());

  if(loadOK) {
    MyMultiBodyCreator creator(0);

    btTransform identityTrans;
    identityTrans.setIdentity();

    ConvertURDF2Bullet2(importer, creator, identityTrans, world, true, importer.getPathPrefix());
    multiBody = creator.getBulletMultiBody();
  }

  RAIINFO(multiBody->getNumLinks())
}

ArticulatedSystem::~ArticulatedSystem() {
}

} // object
} // bulet_sim