//
// Created by kangd on 25.03.18.
//

#include "BtArticulatedSystem.hpp"

namespace bullet_sim {
namespace object {

BtArticulatedSystem::BtArticulatedSystem(std::string urdfFile, btMultiBodyDynamicsWorld *world): dynamicsWorld_(world) {

  urdfFile += "robot.urdf";
  importer_ = new BulletURDFImporter(0, 0, 1.0, CUF_USE_IMPLICIT_CYLINDER | CUF_USE_URDF_INERTIA | CUF_USE_SELF_COLLISION);
  bool loadOK = importer_->loadURDF(urdfFile.c_str());

  if(loadOK) {
    MyMultiBodyCreator creator(0);

    btTransform identityTrans;
    identityTrans.setIdentity();

    ConvertURDF2Bullet2(*importer_, creator, identityTrans, world, true, importer_->getPathPrefix());

    multiBody_ = creator.getBulletMultiBody();
    init();
  }
  else {
    RAIFATAL("failed to load URDF")
  }

  delete importer_;
}

BtArticulatedSystem::~BtArticulatedSystem() {
  delete multiBody_;
}

void BtArticulatedSystem::init() {

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

  genCoordinate_.resize(stateDimension_);
  genCoordinate_.setZero();
  genVelocity_.resize(dof_);
  genVelocity_.setZero();
  genForce_.resize(dof_);
  genForce_.setZero();

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

void BtArticulatedSystem::initVisuals() {

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

void BtArticulatedSystem::initVisualFromLinkCollider(btMultiBodyLinkCollider *linkCollider, int colliderId) {

  // shape
  if (linkCollider->getCollisionShape()->isCompound()) {
    // compound shape
    btCompoundShape *compoundShape =
        (btCompoundShape *) linkCollider->getCollisionShape();

    if(compoundShape->getNumChildShapes() > 0) {
      initVisualFromCompoundChildList(compoundShape->getChildList(),
                                      colliderId,
                                      compoundShape->getNumChildShapes());
      initVisualFromVisualShape(colliderId);
    }
  }
  else {
    // single shape
    initVisualFromCollisionShape(linkCollider->getCollisionShape(),
                                 linkCollider->getWorldTransform(),
                                 colliderId);
    initVisualFromVisualShape(colliderId);
  }
}

void BtArticulatedSystem::initVisualFromCompoundChildList(btCompoundShapeChild *compoundShapeChild,
                                                          int id,
                                                          int numChild) {
  for (int i = 0; i < numChild; ++i) {
    btTransform childTransform = compoundShapeChild[i].m_transform;
    initVisualFromCollisionShape(compoundShapeChild[i].m_childShape, childTransform, id);
  }
}

void BtArticulatedSystem::initVisualFromCollisionShape(btCollisionShape *col, btTransform transform, int id) {

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

  switch (col->getShapeType()) {
    case BOX_SHAPE_PROXYTYPE: {
      // box (xlen, ylen, zlen)
      benchmark::Vec<4> boxSize;
      boxSize = {((btBoxShape *)col)->getHalfExtentsWithMargin().x() * 2.0,
                 ((btBoxShape *)col)->getHalfExtentsWithMargin().y() * 2.0,
                 ((btBoxShape *)col)->getHalfExtentsWithMargin().z() * 2.0,
                 0};

      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Box));
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

      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Cylinder));
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

      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Sphere));
      visColProps_.emplace_back(std::make_pair("", sphereSize));
      break;
    }
    default:
      break;
  }
}

void BtArticulatedSystem::initVisualFromVisualShape(int id) {

  std::vector<URDFVisualData> list;
  importer_->getLinkVisualData(id, list);

  for(int i = 0;  i < list.size(); i++) {

    benchmark::Mat<3,3> mat;
    btMatrix3x3 rotMat;
    rotMat.setRotation(list[i].visual.m_linkLocalFrame.getRotation());
    mat.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
        rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
        rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();

    benchmark::Vec<3> pos;
    pos = {
        list[i].visual.m_linkLocalFrame.getOrigin().x(),
        list[i].visual.m_linkLocalFrame.getOrigin().y(),
        list[i].visual.m_linkLocalFrame.getOrigin().z()
    };

    switch(list[i].visual.m_geometry.m_type) {
      case URDF_GEOM_SPHERE: {
        benchmark::Vec<4> sphereSize;
        sphereSize = {list[i].visual.m_geometry.m_sphereRadius,
                      0,
                      0,
                      0};
        visObj.emplace_back(std::make_tuple(mat,
                                            pos,
                                            id,
                                            benchmark::object::Shape::Sphere,
                                            color_));
        visProps_.emplace_back(std::make_pair("", sphereSize));
        break;
      }
      case URDF_GEOM_BOX: {
        benchmark::Vec<4> boxSize;
        boxSize = {list[i].visual.m_geometry.m_boxSize.x(),
                   list[i].visual.m_geometry.m_boxSize.y(),
                   list[i].visual.m_geometry.m_boxSize.z(),
                   0};
        visObj.emplace_back(std::make_tuple(mat,
                                            pos,
                                            id,
                                            benchmark::object::Shape::Box,
                                            color_));
        visProps_.emplace_back(std::make_pair("", boxSize));
        break;
      }
      case URDF_GEOM_CYLINDER: {
        benchmark::Vec<4> cylSize;
        cylSize = {list[i].visual.m_geometry.m_capsuleRadius,
                   list[i].visual.m_geometry.m_capsuleHeight,
                   0,
                   0};
        visObj.emplace_back(std::make_tuple(mat,
                                            pos,
                                            id,
                                            benchmark::object::Shape::Cylinder,
                                            color_));
        visProps_.emplace_back(std::make_pair("", cylSize));
        break;
      }
      case URDF_GEOM_MESH: {
        benchmark::Vec<4> scale;
        scale = {list[i].visual.m_geometry.m_meshScale.x(),
                 list[i].visual.m_geometry.m_meshScale.y(),
                 list[i].visual.m_geometry.m_meshScale.z(),
                 0};
        visObj.emplace_back(std::make_tuple(mat,
                                            pos,
                                            id,
                                            benchmark::object::Shape::Mesh,
                                            color_));
        visProps_.emplace_back(std::make_pair(list[i].visual.m_geometry.m_meshFileName, scale));
        break;
      }
      default:
        RAIFATAL("not supported visual type")
    }
  }
}

const BtArticulatedSystem::EigenVec BtArticulatedSystem::getGeneralizedCoordinate() {
  if (isFixed_) {
    // fixed body
    int i = 0;
    for (int l: movableLinkIdx_) {
      genCoordinate_[i++] = multiBody_->getJointPos(l);
    }
  }
  else {
    // floating body
    genCoordinate_[0] = multiBody_->getBaseWorldTransform().getOrigin().x();
    genCoordinate_[1] = multiBody_->getBaseWorldTransform().getOrigin().y();
    genCoordinate_[2] = multiBody_->getBaseWorldTransform().getOrigin().z();
    genCoordinate_[3] = multiBody_->getBaseWorldTransform().getRotation().w();
    genCoordinate_[4] = multiBody_->getBaseWorldTransform().getRotation().x();
    genCoordinate_[5] = multiBody_->getBaseWorldTransform().getRotation().y();
    genCoordinate_[6] = multiBody_->getBaseWorldTransform().getRotation().z();

    int i = 7;
    for (int l: movableLinkIdx_) {
      genCoordinate_[i++] = multiBody_->getJointPos(l);
    }
  }
  return genCoordinate_.e();
}

const BtArticulatedSystem::EigenVec BtArticulatedSystem::getGeneralizedVelocity() {
  if (multiBody_->hasFixedBase()) {
    // fixed body
    int i = 0;
    for (int l: movableLinkIdx_) {
      genVelocity_[i++] = multiBody_->getJointVel(l);
    }
  } else {
    // floating body
    genVelocity_[0] = multiBody_->getBaseVel().x();
    genVelocity_[1] = multiBody_->getBaseVel().y();
    genVelocity_[2] = multiBody_->getBaseVel().z();
    genVelocity_[3] = multiBody_->getBaseOmega().x();
    genVelocity_[4] = multiBody_->getBaseOmega().y();
    genVelocity_[5] = multiBody_->getBaseOmega().z();

    int i = 6;
    for (int l: movableLinkIdx_) {
      genVelocity_[i++] = multiBody_->getJointVel(l);
    }
  }
  return genVelocity_.e();
}

void BtArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  RAIFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input")

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

void BtArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")

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

void BtArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  RAIFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input")

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

void BtArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")

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

const BtArticulatedSystem::EigenVec BtArticulatedSystem::getGeneralizedForce() {
  if (multiBody_->hasFixedBase()) {
    // fixed body
    int i = 0;
    for (int l: movableLinkIdx_) {
      genForce_[i++] = multiBody_->getJointTorque(l);
    }
  } else {
    // floating body
    genForce_[0] = multiBody_->getBaseForce().x();
    genForce_[1] = multiBody_->getBaseForce().y();
    genForce_[2] = multiBody_->getBaseForce().z();
    genForce_[3] = multiBody_->getBaseTorque().x();
    genForce_[4] = multiBody_->getBaseTorque().y();
    genForce_[5] = multiBody_->getBaseTorque().z();

    int i = 6;
    for (int l: movableLinkIdx_) {
      genForce_[i++] = multiBody_->getJointTorque(l);
    }
  }
}

void BtArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")

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

void BtArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")

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

void BtArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  RAIINFO("not implemented yet")
}

void BtArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  RAIINFO("not implemented yet")
}

int BtArticulatedSystem::getDOF() {
  return dof_;
}

int BtArticulatedSystem::getStateDimension() {
  return stateDimension_;
}
void BtArticulatedSystem::setColor(Eigen::Vector4d color) {
  color_ = {
      color[0], color[1], color[2], color[3]};

  for(int i = 0; i < visObj.size(); i++) {
    std::get<4>(visObj[i]) = color_;
  }
}
void BtArticulatedSystem::getBodyPose(int bodyId, benchmark::Mat<3, 3> &orientation, benchmark::Vec<3> &position) {

  if(bodyId == 0) {
    // base
    // TODO this is not link's pose but the inertial of the link's pose
    btTransform tf = multiBody_->getBaseWorldTransform();
    orientation.e() << tf.getBasis().getRow(0).x(), tf.getBasis().getRow(0).y(), tf.getBasis().getRow(0).z(),
        tf.getBasis().getRow(1).x(), tf.getBasis().getRow(1).y(), tf.getBasis().getRow(1).z(),
        tf.getBasis().getRow(2).x(), tf.getBasis().getRow(2).y(), tf.getBasis().getRow(2).z();

    position = {tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z()};
  }
  else {
    // non-base
    btTransform linkToInertial;
    linkToInertial.setRotation(multiBody_->getLink(bodyId-1).m_zeroRotParentToThis);
    linkToInertial.setOrigin(multiBody_->getLink(bodyId-1).m_dVector);

    btTransform tf = multiBody_->getLinkCollider(bodyId-1)->getWorldTransform();
    btTransform bodyTf = tf * linkToInertial.inverse();

    orientation.e() << bodyTf.getBasis().getRow(0).x(), bodyTf.getBasis().getRow(0).y(), bodyTf.getBasis().getRow(0).z(),
        bodyTf.getBasis().getRow(1).x(), bodyTf.getBasis().getRow(1).y(), bodyTf.getBasis().getRow(1).z(),
        bodyTf.getBasis().getRow(2).x(), bodyTf.getBasis().getRow(2).y(), bodyTf.getBasis().getRow(2).z();

    position = {bodyTf.getOrigin().x(),
                bodyTf.getOrigin().y(),
                bodyTf.getOrigin().z()};
  }
}
const Eigen::Map<Eigen::Matrix<double, 3, 1>> BtArticulatedSystem::getLinearMomentumInCartesianSpace() {
  RAIFATAL("not implemented")
  linearMomentum_.setZero();
  return linearMomentum_.e();
}

double BtArticulatedSystem::getTotalMass() {
  double mass = multiBody_->getBaseMass();
  for(int i = 0; i < multiBody_->getNumLinks(); i++)
    mass += multiBody_->getLinkMass(i);
  return mass;
}

} // object
} // bulet_sim
