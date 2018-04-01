//
// Created by kangd on 25.03.18.
//

#include "ArticulatedSystem.hpp"

namespace bullet_sim {
namespace object {

ArticulatedSystem::ArticulatedSystem(std::string urdfFile, btMultiBodyDynamicsWorld *world): dynamicsWorld_(world) {

  BulletURDFImporter importer(0, 0, 1.0, CUF_USE_IMPLICIT_CYLINDER | CUF_USE_URDF_INERTIA);
  bool loadOK = importer.loadURDF(urdfFile.c_str());

  if(loadOK) {
    MyMultiBodyCreator creator(0);

    btTransform identityTrans;
    identityTrans.setIdentity();

    ConvertURDF2Bullet2(importer, creator, identityTrans, world, true, importer.getPathPrefix());

    multiBody_ = creator.getBulletMultiBody();
    init();
  }
  else {
    RAIFATAL("failed to load URDF")
  }
}

ArticulatedSystem::~ArticulatedSystem() {

  // delete multibody
  delete multiBody_;
}

void ArticulatedSystem::init() {

  if(multiBody_->hasFixedBase()) {
    isFixed_ = true;
    dof_ = multiBody_->getNumDofs();
    stateDimension_ = dof_;
  }
  else {
    isFixed_ = false;
    dof_ = multiBody_->getNumDofs() + 6;
    stateDimension_ = dof_ + 1;
  }

  jointState_.resize(stateDimension_);
  jointState_.setZero();
  jointVel_.resize(dof_);
  jointVel_.setZero();
  jointForce_.resize(dof_);
  jointForce_.setZero();

  // find movable links
  movableLinkIdx_.reserve(multiBody_->getNumDofs());

  for (int i = 0; i < multiBody_->getNumLinks(); i++) {
    switch(multiBody_->getLink(i).m_jointType) {
      case btMultibodyLink::eRevolute: {
        // 1 DOF
        movableLinkIdx_.push_back(i);
        break;
      }
      case btMultibodyLink::ePrismatic: {
        // 1 DOF
        movableLinkIdx_.push_back(i);
        break;
      }
      case btMultibodyLink::eSpherical: {
        RAIFATAL("spherical joint is not supported")
        break;
      }
      case btMultibodyLink::ePlanar: {
        RAIFATAL("planar joint is not supported")
        break;
      }
      case btMultibodyLink::eFixed:
        // do nothing
        break;
      case btMultibodyLink::eInvalid: {
        RAIFATAL("invalid joint type")
        break;
      }
      default: {
        RAIFATAL("invalid joint type")
        break;
      }
    }
  }

  initVisuals();
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
                                      linkCollider->getWorldTransform(),
                                      colliderId,
                                      compoundShape->getNumChildShapes());
    }
//    else {
//      initVisualFromCollisionShape(linkCollider->getCollisionShape(),
//                                   linkCollider->getWorldTransform().getRotation(),
//                                   linkCollider->getWorldTransform().getOrigin(),
//                                   colliderId);
//    }
  }
  else {

    // single shape
    initVisualFromCollisionShape(linkCollider->getCollisionShape(),
                                 linkCollider->getWorldTransform(),
                                 colliderId);
  }
}

void ArticulatedSystem::initVisualFromCompoundChildList(btCompoundShapeChild *compoundShapeChild,
                                                        btTransform parentTransform,
                                                        int id,
                                                        int numChild) {
  for (int i = 0; i < numChild; ++i) {
    btTransform childTransform = parentTransform * compoundShapeChild[i].m_transform;
    initVisualFromCollisionShape(compoundShapeChild[i].m_childShape, childTransform, id);
  }
}

void ArticulatedSystem::initVisualFromCollisionShape(btCollisionShape *col, btTransform transform, int id) {

  // orientation
  benchmark::Mat<3, 3> mat;
  btMatrix3x3 rotMat;
  rotMat.setRotation(transform.getRotation());
  mat.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
      rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
      rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();

  // position
  benchmark::Vec<3> position;
  position = {transform.getOrigin().x(),
              transform.getOrigin().y(),
              transform.getOrigin().z()};

  // color
  benchmark::Vec<4> color;
  color = {1.0, 0, 0, 1.0};

  switch (col->getShapeType()) {
    case BOX_SHAPE_PROXYTYPE: {
      // box (xlen, ylen, zlen)
      benchmark::Vec<4> boxSize;
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
      benchmark::Vec<4> cylSize;
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
      benchmark::Vec<4> sphereSize;
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

const ArticulatedSystem::EigenVec ArticulatedSystem::getGeneralizedCoordinate() {
  if (isFixed_) {
    // fixed body
    int i = 0;
    for (int l: movableLinkIdx_) {
      jointState_[i++] = multiBody_->getJointPos(l);
    }
  }
  else {
    // floating body
    jointState_[0] = multiBody_->getBaseWorldTransform().getOrigin().x();
    jointState_[1] = multiBody_->getBaseWorldTransform().getOrigin().y();
    jointState_[2] = multiBody_->getBaseWorldTransform().getOrigin().z();
    jointState_[3] = multiBody_->getBaseWorldTransform().getRotation().w();
    jointState_[4] = multiBody_->getBaseWorldTransform().getRotation().x();
    jointState_[5] = multiBody_->getBaseWorldTransform().getRotation().y();
    jointState_[6] = multiBody_->getBaseWorldTransform().getRotation().z();

    int i = 7;
    for (int l: movableLinkIdx_) {
      jointState_[i++] = multiBody_->getJointPos(l);
    }
  }
  return jointState_.e();
}

const ArticulatedSystem::EigenVec ArticulatedSystem::getGeneralizedVelocity() {
  if (multiBody_->hasFixedBase()) {
    // fixed body
    int i = 0;
    for (int l: movableLinkIdx_) {
      jointVel_[i++] = multiBody_->getJointVel(l);
    }
  } else {
    // floating body
    jointVel_[0] = multiBody_->getBaseVel().x();
    jointVel_[1] = multiBody_->getBaseVel().y();
    jointVel_[2] = multiBody_->getBaseVel().z();
    jointVel_[3] = multiBody_->getBaseOmega().x();
    jointVel_[4] = multiBody_->getBaseOmega().y();
    jointVel_[5] = multiBody_->getBaseOmega().z();

    int i = 6;
    for (int l: movableLinkIdx_) {
      jointVel_[i++] = multiBody_->getJointVel(l);
    }
  }
  return jointVel_.e();
}

void ArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  if(isFixed_) {
    // fixed
    int i = 0;
    for (int l: movableLinkIdx_) {
      multiBody_->setJointPos(l, jointState[i++]);
    }
  } else {
    // floating
    multiBody_->setBasePos(btVector3(
        jointState[0],
        jointState[1],
        jointState[2]));
    multiBody_->setWorldToBaseRot(btQuaternion(
        jointState[4],
        jointState[5],
        jointState[6],
        jointState[3]));

    int i = 7;
    for (int l: movableLinkIdx_) {
      multiBody_->setJointPos(l, jointState[i++]);
    }
  }
}

void ArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  if(isFixed_) {
    // fixed
    int i = 0;
    for (int l: movableLinkIdx_) {
      multiBody_->setJointPos(l, jointVel[i++]);
    }
  } else {
    // floating
    multiBody_->setBaseVel(btVector3(
        jointVel[0],
        jointVel[1],
        jointVel[2]));
    multiBody_->setBaseOmega(btVector3(
        jointVel[3],
        jointVel[4],
        jointVel[5]));

    int i = 6;
    for (int l: movableLinkIdx_) {
      multiBody_->setJointPos(l, jointVel[i++]);
    }
  }
}

void ArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  if(isFixed_) {
    // fixed
    int i = 0;
    for (int l: movableLinkIdx_) {
      multiBody_->setJointPos(l, jointState.begin()[i]);
    }
  } else {
    // floating
    multiBody_->setBasePos(btVector3(
        jointState.begin()[0],
        jointState.begin()[1],
        jointState.begin()[2]));
    multiBody_->setWorldToBaseRot(btQuaternion(
        jointState.begin()[4],
        jointState.begin()[5],
        jointState.begin()[6],
        jointState.begin()[3]));

    int i = 7;
    for (int l: movableLinkIdx_) {
      multiBody_->setJointPos(l, jointState.begin()[i]);
    }
  }
}

void ArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  if(isFixed_) {
    // fixed
    int i = 0;
    for (int l: movableLinkIdx_) {
      multiBody_->setJointPos(l, jointVel.begin()[i++]);
    }
  } else {
    // floating
    multiBody_->setBaseVel(btVector3(
        jointVel.begin()[0],
        jointVel.begin()[1],
        jointVel.begin()[2]));
    multiBody_->setBaseOmega(btVector3(
        jointVel.begin()[3],
        jointVel.begin()[4],
        jointVel.begin()[5]));

    int i = 6;
    for (int l: movableLinkIdx_) {
      multiBody_->setJointPos(l, jointVel.begin()[i++]);
    }
  }
}

const ArticulatedSystem::EigenVec ArticulatedSystem::getGeneralizedForce() {
  if (multiBody_->hasFixedBase()) {
    // fixed body
    int i = 0;
    for (int l: movableLinkIdx_) {
      jointForce_[i++] = multiBody_->getJointTorque(l);
    }
  } else {
    // floating body
    jointForce_[0] = multiBody_->getBaseForce().x();
    jointForce_[1] = multiBody_->getBaseForce().y();
    jointForce_[2] = multiBody_->getBaseForce().z();
    jointForce_[3] = multiBody_->getBaseTorque().x();
    jointForce_[4] = multiBody_->getBaseTorque().y();
    jointForce_[5] = multiBody_->getBaseTorque().z();

    int i = 6;
    for (int l: movableLinkIdx_) {
      jointForce_[i++] = multiBody_->getJointTorque(l);
    }
  }
}

void ArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {
  if(isFixed_) {
    // fixed
    int i = 0;
    for (int l: movableLinkIdx_) {
      multiBody_->addJointTorque(l, tau.begin()[i++]);
    }
  } else {
    // floating
    multiBody_->addBaseForce(btVector3(
        tau.begin()[0],
        tau.begin()[1],
        tau.begin()[2]));
    multiBody_->addBaseTorque(btVector3(
        tau.begin()[3],
        tau.begin()[4],
        tau.begin()[5]));

    int i = 6;
    for (int l: movableLinkIdx_) {
      multiBody_->addJointTorque(l, tau.begin()[i++]);
    }
  }
}

void ArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {
  if(isFixed_) {
    // fixed
    int i = 0;
    for (int l: movableLinkIdx_) {
      multiBody_->addJointTorque(l, tau[i++]);
    }
  } else {
    // floating
    multiBody_->addBaseForce(btVector3(
        tau[0],
        tau[1],
        tau[2]));
    multiBody_->addBaseTorque(btVector3(
        tau[3],
        tau[4],
        tau[5]));

    int i = 6;
    for (int l: movableLinkIdx_) {
      multiBody_->addJointTorque(l, tau[i++]);
    }
  }
}

void ArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {

}

void ArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {

}

int ArticulatedSystem::getDOF() {
  return dof_;
}

} // object
} // bulet_sim
