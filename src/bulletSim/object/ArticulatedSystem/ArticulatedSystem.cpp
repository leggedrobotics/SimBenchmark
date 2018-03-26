//
// Created by kangd on 25.03.18.
//

#include <raiSim/math.hpp>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
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

    for(int i = 0; i < multiBody->getNumLinks(); i++) {
      btMultiBodyLinkCollider *linkCollider = multiBody->getLinkCollider(i);

      // orientation
      rai_sim::Mat<3,3> mat;
      btMatrix3x3 rotMat = linkCollider->getWorldTransform().getBasis();
      mat.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
          rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
          rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();

      // position
      rai_sim::Vec<3> pos;
      btVector3 position = linkCollider->getWorldTransform().getOrigin();
      pos = {position.x(), position.y(), position.z()};

      // color
      rai_sim::Vec<4> color;
      color = {1.0, 0, 0, 1.0};

      // shape
      if(linkCollider->getCollisionShape()->isCompound()) {
        // compound shape
        btCompoundShape *compoundShape = (btCompoundShape *)multiBody->getLinkCollider(i)->getCollisionShape();

        if(compoundShape->getNumChildShapes() > 1) {
          // multiple children shapes
          // TODO
        } else if (compoundShape->getNumChildShapes() == 1) {
          // only one child shape
          btCollisionShape *col = compoundShape->getChildShape(0);
          collisionShapes.push_back(col);

          RAIINFO(col->getName())
          switch (col->getShapeType()) {
            case 0: {
              // box (xlen, ylen, zlen)
              RAIINFO("box")
              rai_sim::Vec<4> boxSize;
              boxSize = {((btBoxShape *)col)->getHalfExtentsWithMargin().x() * 2.0,
                         ((btBoxShape *)col)->getHalfExtentsWithMargin().y() * 2.0,
                         ((btBoxShape *)col)->getHalfExtentsWithMargin().z() * 2.0,
                         0};
              RAIINFO(boxSize.e())

              visObj.emplace_back(std::make_tuple(mat, pos, i, benchmark::object::Shape::Box, color));
              visColObj.emplace_back(std::make_tuple(mat, pos, i, benchmark::object::Shape::Box));
              visProps_.emplace_back(std::make_pair("", boxSize));
              visColProps_.emplace_back(std::make_pair("", boxSize));
              break;
            }
            case 1:
              break;
            case 2:
              break;
            case 3:
              break;
            case 4: {
              // cylinder (rad, rad, height)
              RAIINFO("cylinder")
              rai_sim::Vec<4> cylSize;
              cylSize = {((btCylinderShapeZ *)col)->getHalfExtentsWithoutMargin().y(),
                         ((btCylinderShapeZ *)col)->getHalfExtentsWithoutMargin().z() * 2.0,
                         0,
                         0};
              RAIINFO(cylSize.e())

              visObj.emplace_back(std::make_tuple(mat, pos, i, benchmark::object::Shape::Cylinder, color));
              visColObj.emplace_back(std::make_tuple(mat, pos, i, benchmark::object::Shape::Cylinder));
              visProps_.emplace_back(std::make_pair("", cylSize));
              visColProps_.emplace_back(std::make_pair("", cylSize));
              break;
            }
            case 5:
              break;
            case 6:
              break;
            case 7:
              break;
            case 8: {
              // sphere
              RAIINFO("sphere")
              rai_sim::Vec<4> sphereSize;
              sphereSize = {((btSphereShape *)col)->getRadius(),
                            0,
                            0,
                            0};
              RAIINFO(sphereSize.e())

              visObj.emplace_back(std::make_tuple(mat, pos, i, benchmark::object::Shape::Sphere, color));
              visColObj.emplace_back(std::make_tuple(mat, pos, i, benchmark::object::Shape::Sphere));
              visProps_.emplace_back(std::make_pair("", sphereSize));
              visColProps_.emplace_back(std::make_pair("", sphereSize));
              break;
            }
            default:
              break;
          }
        }
      }
      else {
        // single shape
        btCollisionShape *col = multiBody->getLinkCollider(i)->getCollisionShape();
        collisionShapes.push_back(col);
      }
    }
  }
}

ArticulatedSystem::~ArticulatedSystem() {
}

const std::vector<btCollisionShape *> &ArticulatedSystem::getCollisionShapes() const {
  return collisionShapes;
}

} // object
} // bulet_sim
