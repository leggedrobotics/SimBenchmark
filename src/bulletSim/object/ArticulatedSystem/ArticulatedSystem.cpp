//
// Created by kangd on 25.03.18.
//

#include <raiSim/math.hpp>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include "ArticulatedSystem.hpp"

namespace bullet_sim {
namespace object {

ArticulatedSystem::ArticulatedSystem(std::string urdfFile, btMultiBodyDynamicsWorld *world): dynamicsWorld_(world) {

  importer_ = new BulletURDFImporter(0, 0, 1.0, CUF_USE_IMPLICIT_CYLINDER | CUF_USE_URDF_INERTIA);
  bool loadOK = importer_->loadURDF(urdfFile.c_str());

  if(loadOK) {
    creator_ = new MyMultiBodyCreator(0);

    btTransform identityTrans;
    identityTrans.setIdentity();

    ConvertURDF2Bullet2(*importer_, *creator_, identityTrans, world, true, importer_->getPathPrefix());

    multiBody_ = creator.getBulletMultiBody();
    init();
    multiBody_ = creator_->getBulletMultiBody();
    initVisuals();
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
    initVisColObjFromLinkCollider(baseCollider, 0);
  }

  // link
  for (int i = 0; i < multiBody_->getNumLinks(); i++) {
    btMultiBodyLinkCollider *linkCollider = multiBody_->getLinkCollider(i);
    initVisColObjFromLinkCollider(linkCollider, i + 1);

    std::vector<URDFVisualData> visDataList = importer_->getLinkVisualData(creator_->m_mb2urdfLink[i]);
    initVisObj(multiBody_->getLink(i).m_cachedWorldTransform, visDataList);
  }
}

void ArticulatedSystem::initVisObj(btTransform linkTransform, std::vector<URDFVisualData> &data) {

  rai_sim::Vec<4> color;
  color = {1.0, 0, 0, 1.0};

  for (int i = 0; i < data.size(); ++i) {
    UrdfVisual &vis = data[i].visual;

    // orientation
    rai_sim::Mat<3, 3> mat;
    btMatrix3x3 rotMat;
    rotMat.setRotation(
        linkTransform.getRotation() /* *vis.m_linkLocalFrame.getRotation()*/);
    mat.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
        rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
        rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();

    // position
    rai_sim::Vec<3> position;
    btVector3 pos = linkTransform.getOrigin() /*+ vis.m_linkLocalFrame.getOrigin()*/;
    position = {pos.x(),
                pos.y(),
                pos.z()};

    switch (vis.m_geometry.m_type) {
      case URDF_GEOM_MESH:
        rai_sim::Vec<4> scale;
        scale = {
            vis.m_geometry.m_meshScale.x(),
            vis.m_geometry.m_meshScale.y(),
            vis.m_geometry.m_meshScale.z(),
            0
        };
        visObj.emplace_back(std::make_tuple(mat, position, 0, benchmark::object::Shape::Mesh, color));
        visProps_.emplace_back(std::make_pair(vis.m_geometry.m_meshFileName, scale));
        break;
      case URDF_GEOM_BOX:
        rai_sim::Vec<4> boxSize;
        boxSize = {vis.m_geometry.m_boxSize.x(),
                   vis.m_geometry.m_boxSize.y(),
                   vis.m_geometry.m_boxSize.z(),
                   0};
        visObj.emplace_back(std::make_tuple(mat, position, 0, benchmark::object::Shape::Box, color));
        visProps_.emplace_back(std::make_pair("", boxSize));
        break;
      case URDF_GEOM_CYLINDER:
        rai_sim::Vec<4> cylSize;
        boxSize = {vis.m_geometry.m_capsuleRadius,
                   vis.m_geometry.m_capsuleHeight,
                   0,
                   0};
        visObj.emplace_back(std::make_tuple(mat, position, 0, benchmark::object::Shape::Cylinder, color));
        visProps_.emplace_back(std::make_pair("", cylSize));
        break;
      case URDF_GEOM_SPHERE:
        rai_sim::Vec<4> sphereSize;
        boxSize = {vis.m_geometry.m_sphereRadius,
                   0,
                   0,
                   0};
        visObj.emplace_back(std::make_tuple(mat, position, 0, benchmark::object::Shape::Sphere, color));
        visProps_.emplace_back(std::make_pair("", sphereSize));
        break;
      case URDF_GEOM_CAPSULE:
      case URDF_GEOM_PLANE:
      case URDF_GEOM_UNKNOWN:
      default:
        break;
    }
  }
}

void ArticulatedSystem::initVisColObjFromLinkCollider(btMultiBodyLinkCollider *linkCollider, int colliderId) {

  // shape
  if (linkCollider->getCollisionShape()->isCompound()) {

    // compound shape
    btCompoundShape *compoundShape =
        (btCompoundShape *) linkCollider->getCollisionShape();

    if(compoundShape->getNumChildShapes() > 0) {
      initVisColObjFromCompoundChildList(compoundShape->getChildList(),
                                         linkCollider->getWorldTransform().getRotation(),
                                         linkCollider->getWorldTransform().getOrigin(),
                                         colliderId,
                                         compoundShape->getNumChildShapes());
    }
  }
  else {

    // single shape
    initVisColObjFromCollisionShape(linkCollider->getCollisionShape(),
                                    linkCollider->getWorldTransform().getRotation(),
                                    linkCollider->getWorldTransform().getOrigin(),
                                    colliderId);
  }
}
void ArticulatedSystem::initVisColObjFromCompoundChildList(btCompoundShapeChild *compoundShapeChild,
                                                           btQuaternion parentQuat,
                                                           btVector3 parentPos,
                                                           int id,
                                                           int numChild) {
  for (int i = 0; i < numChild; ++i) {
    btQuaternion childquat = parentQuat * compoundShapeChild[i].m_transform.getRotation();
    btVector3 childpos = parentPos + compoundShapeChild[i].m_transform.getOrigin();
    initVisColObjFromCollisionShape(compoundShapeChild[i].m_childShape, childquat, childpos, id);
  }
}

void ArticulatedSystem::initVisColObjFromCollisionShape(btCollisionShape *col,
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

//      visObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Box, color));
      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Box));
//      visProps_.emplace_back(std::make_pair("", boxSize));
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

//      visObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Cylinder, color));
      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Cylinder));
//      visProps_.emplace_back(std::make_pair("", cylSize));
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

//      visObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Sphere, color));
      visColObj.emplace_back(std::make_tuple(mat, position, id, benchmark::object::Shape::Sphere));
//      visProps_.emplace_back(std::make_pair("", sphereSize));
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
