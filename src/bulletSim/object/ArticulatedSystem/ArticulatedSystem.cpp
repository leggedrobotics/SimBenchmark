//
// Created by kangd on 25.03.18.
//

#include <raiSim/math.hpp>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include "ArticulatedSystem.hpp"

namespace bullet_sim {
namespace object {

ArticulatedSystem::ArticulatedSystem(std::string urdfFile, btMultiBodyDynamicsWorld *world) {

  urdfImporter_ = new BulletURDFImporter(0, 0, 1.0, CUF_USE_IMPLICIT_CYLINDER | CUF_USE_URDF_INERTIA);
  bool loadOK = urdfImporter_->loadURDF(urdfFile.c_str());

  if(loadOK) {
    MyMultiBodyCreator creator(0);

    btTransform identityTrans;
    identityTrans.setIdentity();
    identityTrans.setOrigin({5, 5, 5});

    ConvertURDF2Bullet2(*urdfImporter_, creator, identityTrans, world, true, urdfImporter_->getPathPrefix());

    multiBody_ = creator.getBulletMultiBody();
    initVisuals();
  }
  else {
    RAIFATAL("failed to load URDF")
  }
}

ArticulatedSystem::~ArticulatedSystem() {
  delete multiBody_;
  delete urdfImporter_;
}

void ArticulatedSystem::initVisuals() {

  // base
  {
    btMultiBodyLinkCollider *baseCollider = multiBody_->getBaseCollider();
    initVisualFromCollider(baseCollider, 0);
  }

  // link
  for (int i = 0; i < multiBody_->getNumLinks(); i++) {
    btMultiBodyLinkCollider *linkCollider = multiBody_->getLinkCollider(i);

    if(urdfImporter_->getMeshFilePath(i).size() > 0){
      initVisualFromCollider(linkCollider, i + 1, urdfImporter_->getMeshFilePath(i)[0]);
    }
    else
      initVisualFromCollider(linkCollider, i + 1);
  }
}

void ArticulatedSystem::initVisualFromCollider(btMultiBodyLinkCollider *linkCollider, int colliderId, std::string meshfile) {

  // orientation
  rai_sim::Mat<3, 3> mat;
  btQuaternion quat = linkCollider->getWorldTransform().getRotation();
  btMatrix3x3 rotMat;
  rotMat.setRotation(quat);
  mat.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
      rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
      rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();

  // position
  rai_sim::Vec<3> pos;
  btVector3 position = linkCollider->getWorldTransform().getOrigin();
  pos = {position.x(), position.y(), position.z()};

  // shape
  if (linkCollider->getCollisionShape()->isCompound()) {
    // compound shape
    btCompoundShape *compoundShape =
        (btCompoundShape *)linkCollider->getCollisionShape();

    if (compoundShape->getNumChildShapes() > 1) {
      // multiple children shapes TODO
    } else if (compoundShape->getNumChildShapes() == 1) {
      // only one child shape
      btCollisionShape *col = compoundShape->getChildShape(0);
      initVisualFromCollisionShape(col, mat, pos, colliderId);
    }
    else {
      // single shape
      btCollisionShape *col = linkCollider->getCollisionShape();
      initVisualFromCollisionShape(col, mat, pos, colliderId);
    }
  }
}

void ArticulatedSystem::initVisualFromCollisionShape(btCollisionShape *col,
                                                     rai_sim::Mat<3, 3> rotMat,
                                                     rai_sim::Vec<3> pos,
                                                     int id,
                                                     std::string meshfile) {

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

      if(meshfile.empty()) {
        visObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Box, color));
        visProps_.emplace_back(std::make_pair("", boxSize));
      }
      else {
        visObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Mesh, color));
        visProps_.emplace_back(std::make_pair(meshfile, boxSize));
      }
      visColObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Box));
      visColProps_.emplace_back(std::make_pair("", boxSize));
      break;
    }
    case CYLINDER_SHAPE_PROXYTYPE: {
      // cylinder (rad, rad, height)
      rai_sim::Vec<4> cylSize;
      cylSize = {((btCylinderShapeZ *)col)->getHalfExtentsWithoutMargin().x(),
                 ((btCylinderShapeZ *)col)->getHalfExtentsWithoutMargin().z() * 2.0,
                 0,
                 0};

      if(meshfile.empty()) {
        visObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Cylinder, color));
        visProps_.emplace_back(std::make_pair("", cylSize));
      }
      else {
        visObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Mesh, color));
        visProps_.emplace_back(std::make_pair(meshfile, cylSize));
      }
      visColObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Cylinder));
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

      if(meshfile.empty()) {
        visObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Sphere, color));
        visProps_.emplace_back(std::make_pair("", sphereSize));
      }
      else {
        visObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Mesh, color));
        visProps_.emplace_back(std::make_pair(meshfile, sphereSize));
      }
      visColObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Sphere));
      visColProps_.emplace_back(std::make_pair("", sphereSize));
      break;
    }
    default: {
      // cylinder (rad, rad, height)
      rai_sim::Vec<4> cylSize;
      cylSize = {((btCylinderShapeZ *)col)->getHalfExtentsWithoutMargin().x(),
                 ((btCylinderShapeZ *)col)->getHalfExtentsWithoutMargin().z() * 2.0,
                 0,
                 0};

      if(meshfile.empty()) {
        visObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Cylinder, color));
        visProps_.emplace_back(std::make_pair("", cylSize));
      }
      else {
        visObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Mesh, color));
        visProps_.emplace_back(std::make_pair(meshfile, cylSize));
      }
      visColObj.emplace_back(std::make_tuple(rotMat, pos, id, benchmark::object::Shape::Cylinder));
      visColProps_.emplace_back(std::make_pair("", cylSize));
      break;
    }
  }
}
void ArticulatedSystem::updateVisuals() {

  // base
  {
    btMultiBodyLinkCollider *baseCollider = multiBody_->getBaseCollider();
    // orientation
    rai_sim::Mat<3, 3> mat;
    btQuaternion quat = baseCollider->getWorldTransform().getRotation();
    btMatrix3x3 rotMat;
    rotMat.setRotation(quat);
    mat.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
        rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
        rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();

    // position
    rai_sim::Vec<3> pos;
    btVector3 position = baseCollider->getWorldTransform().getOrigin();
    pos = {position.x(), position.y(), position.z()};

    std::get<0>(visObj[0]) = mat;
    std::get<0>(visColObj[0]) = mat;
    std::get<1>(visObj[0]) = pos;
    std::get<1>(visColObj[0]) = pos;
  }

  // links
  for (int i = 0; i < multiBody_->getNumLinks(); i++) {
    btMultiBodyLinkCollider *linkCollider = multiBody_->getLinkCollider(i);

    // orientation
    rai_sim::Mat<3, 3> mat;
    btQuaternion quat = linkCollider->getWorldTransform().getRotation();
    btMatrix3x3 rotMat;
    rotMat.setRotation(quat);
    mat.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
        rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
        rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();

    // position
    rai_sim::Vec<3> pos;
    btVector3 position = linkCollider->getWorldTransform().getOrigin();
    pos = {position.x(), position.y(), position.z()};

    std::get<0>(visObj[i+1]) = mat;
    std::get<0>(visColObj[i+1]) = mat;
    std::get<1>(visObj[i+1]) = pos;
    std::get<1>(visColObj[i+1]) = pos;
  }
}

} // object
} // bulet_sim
