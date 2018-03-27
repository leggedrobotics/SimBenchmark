//
// Created by kangd on 25.03.18.
//

#include <raiSim/math.hpp>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include "ArticulatedSystem.hpp"

namespace bullet_sim {
namespace object {

ArticulatedSystem::ArticulatedSystem(std::string urdfFile, btMultiBodyDynamicsWorld *world) {

  importer_ = new BulletURDFImporter(0, 0, 1.0, CUF_USE_IMPLICIT_CYLINDER | CUF_USE_URDF_INERTIA);
  bool loadOK = importer_->loadURDF(urdfFile.c_str());

  if(loadOK) {
    creator_ = new MyMultiBodyCreator(0);

    btTransform identityTrans;
    identityTrans.setIdentity();
    identityTrans.setOrigin({0, 0, 5});

    ConvertURDF2Bullet2(*importer_, *creator_, identityTrans, world, true, importer_->getPathPrefix());

    multiBody_ = creator_->getBulletMultiBody();
    initVisuals();
  }
  else {
    RAIFATAL("failed to load URDF")
  }
}

ArticulatedSystem::~ArticulatedSystem() {
  delete importer_;
  delete creator_;
  delete multiBody_;
}

void ArticulatedSystem::updateVisuals() {
  visObj.clear();
  visColObj.clear();
  visProps_.clear();
  visColProps_.clear();

  initVisuals();
}

void ArticulatedSystem::initVisuals() {

  // reserve
  visObj.reserve(multiBody_->getNumLinks() + 1);
  visColObj.reserve(multiBody_->getNumLinks() + 1);
  visProps_.reserve(multiBody_->getNumLinks() + 1);
  visColProps_.reserve(multiBody_->getNumLinks() + 1);

  // base
  {
    btMultiBodyLinkCollider *baseCollider = multiBody_->getBaseCollider();
    initVisualFromLinkCollider(baseCollider, 0);
  }

  // link
  for (int i = 0; i < multiBody_->getNumLinks(); i++) {
    btMultiBodyLinkCollider *linkCollider = multiBody_->getLinkCollider(i);
    initVisualFromLinkCollider(linkCollider, i + 1);

    if(importer_->getLinkMeshData(creator_->m_mb2urdfLink[i]).size() > 0)
      RAIINFO(importer_->getLinkMeshData(creator_->m_mb2urdfLink[i])[0].meshFile_)
  }
}

void ArticulatedSystem::initVisualFromLinkCollider(btMultiBodyLinkCollider *linkCollider, int colliderId) {

  // shape
  if (linkCollider->getCollisionShape()->isCompound()) {

    // compound shape
    btCompoundShape *compoundShape =
        (btCompoundShape *) linkCollider->getCollisionShape();

    if(compoundShape->getNumChildShapes() > 0) {
      initVisualFromCompoundChildList(compoundShape->getChildList(),
                                      linkCollider->getWorldTransform().getRotation(),
                                      linkCollider->getWorldTransform().getOrigin(),
                                      colliderId,
                                      compoundShape->getNumChildShapes());
    }
  }
  else {

    // single shape
    initVisualFromCollisionShape(linkCollider->getCollisionShape(),
                                 linkCollider->getWorldTransform().getRotation(),
                                 linkCollider->getWorldTransform().getOrigin(),
                                 colliderId);
  }
}

void ArticulatedSystem::initVisualFromCompoundChildList(btCompoundShapeChild *compoundShapeChild,
                                                        btQuaternion parentQuat,
                                                        btVector3 parentPos,
                                                        int id,
                                                        int numChild) {
  for (int i = 0; i < numChild; ++i) {
    btQuaternion childquat = parentQuat * compoundShapeChild[i].m_transform.getRotation();
    btVector3 childpos = parentPos + compoundShapeChild[i].m_transform.getOrigin();
    initVisualFromCollisionShape(compoundShapeChild[i].m_childShape, childquat, childpos, id);
  }
}
void ArticulatedSystem::initVisualFromCollisionShape(btCollisionShape *col,
                                                     btQuaternion quat,
                                                     btVector3 pos,
                                                     int id) {

  // orientation
  rai_sim::Mat<3, 3> mat;
  btMatrix3x3 rotMat;
  rotMat.setRotation(quat);
  mat.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
      rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
      rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();

  // position
  rai_sim::Vec<3> position;
  position = {pos.x(), pos.y(), pos.z()};

  // color
  rai_sim::Vec<4> color;
  color = {1.0, 0, 0, 1.0};

  switch (col->getShapeType()) {
    case BOX_SHAPE_PROXYTYPE: {
      // box (xlen, ylen, zlen)
      rai_sim::Vec<4> boxSize;
      boxSize = {((btBoxShape *)col)->getHalfExtentsWithMargin().x() * 2.0,
                 ((btBoxShape *)col)->getHalfExtentsWithMargin().y() * 2.0,
                 ((btBoxShape *)col)->getHalfExtentsWithMargin().z() * 2.0,
                 0};

      visObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Box, color));
      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Box));
      visProps_.emplace_back(std::make_pair("", boxSize));
      visColProps_.emplace_back(std::make_pair("", boxSize));
      break;
    }
    case CYLINDER_SHAPE_PROXYTYPE: {
      // cylinder (rad, rad, height)
      rai_sim::Vec<4> cylSize;
      cylSize = {((btCylinderShapeZ *)col)->getHalfExtentsWithMargin().x(),
                 ((btCylinderShapeZ *)col)->getHalfExtentsWithMargin().z() * 2.0,
                 0,
                 0};

      visObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Cylinder, color));
      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Cylinder));
      visProps_.emplace_back(std::make_pair("", cylSize));
      visColProps_.emplace_back(std::make_pair("", cylSize));
      break;
    }
    case SPHERE_SHAPE_PROXYTYPE: {
      // sphere
      rai_sim::Vec<4> sphereSize;
      sphereSize = {((btSphereShape *)col)->getRadius(),
                    0,
                    0,
                    0};

      visObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Sphere, color));
      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Sphere));
      visProps_.emplace_back(std::make_pair("", sphereSize));
      visColProps_.emplace_back(std::make_pair("", sphereSize));
      break;
    }
    default:
      break;
  }
}

} // object
} // bulet_sim
