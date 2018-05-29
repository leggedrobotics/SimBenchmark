//
// Created by kangd on 25.05.18.
//

#include "BtMbArticulatedSystem.hpp"

namespace bullet_mb_sim {

object::BtMbArticulatedSystem::BtMbArticulatedSystem(std::string filePath,
                                                     ObjectFileType fileType,
                                                     bool internalCollision,
                                                     b3RobotSimulatorClientAPI_NoGUI *api) : api_(api) {

  int objectId = -1;

  // load model file
  switch (fileType) {
    case URDF: {
      b3RobotSimulatorLoadUrdfFileArgs args;
      args.m_flags =
          URDF_USE_INERTIA_FROM_FILE | URDF_USE_IMPLICIT_CYLINDER | MJCF_COLORS_FROM_FILE;

      if(internalCollision)
        args.m_flags |= URDF_USE_SELF_COLLISION;

      objectId = api->loadURDF(filePath + "/robot.urdf", args);
      break;
    }
    case SDF:
    case MJCF:
    RAIFATAL("currently, only URDF is supported")
      break;
    default:
    RAIFATAL("wrong object file type")
      break;
  }

  // set object id
  RAIFATAL_IF(objectId == -1, "cannot load Object")
  objectId_ = objectId;

  // initialize visual data
  b3VisualShapeInformation visualShapeInfo;
  {
    api_->getVisualShapeData(objectId, visualShapeInfo);
    for (int i = 0; i < visualShapeInfo.m_numVisualShapes; i++) {
      initVisuals(objectId, visualShapeInfo.m_visualShapeData[i]);
    }
  }

  // initialize collision data & mass & physical props
  // base (link id = -1)
  {
    b3CollisionShapeInformation collisionShapeInfo;
    api_->getCollisionShapeData(objectId, -1, collisionShapeInfo);
    initCollisions(objectId, -1, collisionShapeInfo);

    b3DynamicsInfo info;
    api_->getDynamicsInfo(objectId, -1, &info);
    mass_.push_back(info.m_mass);

    benchmark::Mat<3,3> inertia;
    inertia.setZero();
    inertia[0] = info.m_localInertialDiagonal[0];
    inertia[4] = info.m_localInertialDiagonal[1];
    inertia[8] = info.m_localInertialDiagonal[2];
    inertia_.push_back(inertia);

    b3RobotSimulatorChangeDynamicsArgs arg;
    arg.m_lateralFriction = 0.8;
    arg.m_linearDamping = 0;
    arg.m_angularDamping = 0;
    arg.m_restitution = 0;
    api_->changeDynamics(objectId, -1, arg);
  }

  // links
  for(int i = 0; i < api->getNumJoints(objectId); i++) {
    b3CollisionShapeInformation collisionShapeInfo;
    api_->getCollisionShapeData(objectId, i, collisionShapeInfo);
    initCollisions(objectId, i, collisionShapeInfo);

    b3DynamicsInfo info;
    api_->getDynamicsInfo(objectId, i, &info);
    mass_.push_back(info.m_mass);

    benchmark::Mat<3,3> inertia;
    inertia.setZero();
    inertia[0] = info.m_localInertialDiagonal[0];
    inertia[4] = info.m_localInertialDiagonal[1];
    inertia[8] = info.m_localInertialDiagonal[2];
    inertia_.push_back(inertia);

    b3RobotSimulatorChangeDynamicsArgs arg;
    arg.m_lateralFriction = 0.8;
    arg.m_linearDamping = 0;
    arg.m_angularDamping = 0;
    api_->changeDynamics(objectId, i, arg);
  }

  // is fixed
  {
    isFixed_ = false;
  }

  // dof, state dim, and motor set
  {
    int numJoint = api_->getNumJoints(objectId);
    if (isFixed_) {
      dof_ = 0;
      stateDimension_ = 0;
    } else {
      dof_ = 6;
      stateDimension_ = 7;
    }

    for (int i = 0; i < numJoint; i++) {
      b3JointInfo info;
      api_->getJointInfo(objectId, i, &info);

      {
        // disable motor of each joints
        b3RobotSimulatorJointMotorArgs arg(CONTROL_MODE_VELOCITY);
        arg.m_maxTorqueValue = 0;
        api_->setJointMotorControl(objectId, i, arg);
      }

      switch(info.m_jointType) {
        case eFixedType:
          break;
        case eRevoluteType:
        case ePrismaticType: {
          ctrbJoints_.push_back(i);
          dof_++;
          stateDimension_++;
          numJoints_++;
          break;
        }
        case eSphericalType:
        case ePlanarType:
        case ePoint2PointType:
        case eGearType:
        default:
        RAIFATAL("not supported joint type")
      }
    }

    genCoordinate_.resize(stateDimension_);
    genCoordinate_.setZero();
    genVelocity_.resize(dof_);
    genVelocity_.setZero();
    genForce_.resize(dof_);
    genForce_.setZero();
  }
}

object::BtMbArticulatedSystem::~BtMbArticulatedSystem() {}

void object::BtMbArticulatedSystem::initVisuals(int objectId, b3VisualShapeData &data) {

  // link id
  int linkId = data.m_linkIndex;

  // orientation
  benchmark::Mat<3, 3> mat;
  benchmark::Vec<4> quat;
  quat[0] = data.m_localVisualFrame[6]; // w
  quat[1] = data.m_localVisualFrame[3]; // x
  quat[2] = data.m_localVisualFrame[4]; // y
  quat[3] = data.m_localVisualFrame[5]; // z
  benchmark::quatToRotMat(quat, mat);

  // position
  benchmark::Vec<3> position;
  position[0] = data.m_localVisualFrame[0];
  position[1] = data.m_localVisualFrame[1];
  position[2] = data.m_localVisualFrame[2];

  // color
  benchmark::Vec<4> color;
  color[0] = data.m_rgbaColor[0];
  color[1] = data.m_rgbaColor[1];
  color[2] = data.m_rgbaColor[2];
  color[3] = data.m_rgbaColor[3];

  switch (data.m_visualGeometryType) {
    case GEOM_SPHERE: {
      benchmark::Vec<4> dim = {
          data.m_dimensions[0],   // radius
          0,
          0,
          0
      };
      visObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Sphere, color));
      visProps_.emplace_back(std::make_pair("", dim));
      break;
    }
    case GEOM_BOX: {
      benchmark::Vec<4> dim = {
          data.m_dimensions[0],
          data.m_dimensions[1],
          data.m_dimensions[2],
          0
      };
      visObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Box, color));
      visProps_.emplace_back(std::make_pair("", dim));
      break;
    }
    case GEOM_CYLINDER: {
      benchmark::Vec<4> dim = {
          data.m_dimensions[1],   // radius
          data.m_dimensions[0],   // length
          0,
          0
      };
      visObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Cylinder, color));
      visProps_.emplace_back(std::make_pair("", dim));
      break;
    }
    case GEOM_MESH: {
      benchmark::Vec<4> dim;
      dim = {data.m_dimensions[0],
             data.m_dimensions[1],
             data.m_dimensions[2],
             0};
      visObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Mesh, color));
      visProps_.emplace_back(std::make_pair(data.m_meshAssetFileName, dim));
      break;
    }
    case GEOM_CAPSULE: {
      benchmark::Vec<4> dim = {
          data.m_dimensions[1],   // radius
          data.m_dimensions[0],   // length
          0
      };
      visObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Capsule, color));
      visProps_.emplace_back(std::make_pair("", dim));
      break;
    }
    case GEOM_PLANE:
    case GEOM_UNKNOWN:
    default:
    RAIFATAL("invalid visual shape")
  }
}

void object::BtMbArticulatedSystem::initCollisions(int objectId, int linkId, b3CollisionShapeInformation &info) {
  for(int i = 0; i < info.m_numCollisionShapes; i++) {
    b3CollisionShapeData &data = info.m_collisionShapeData[i];

    // link id
//    RAIFATAL_IF(data.m_linkIndex != info->m_jointIndex, "joint idx is not same with link idx")
//    int linkId = linkId;

    // orientation
    benchmark::Mat<3, 3> mat;
    benchmark::Vec<4> quat;
    quat[0] = data.m_localCollisionFrame[6];  // w
    quat[1] = data.m_localCollisionFrame[3];  // x
    quat[2] = data.m_localCollisionFrame[4];  // y
    quat[3] = data.m_localCollisionFrame[5];  // z
    benchmark::quatToRotMat(quat, mat);

    // position
    benchmark::Vec<3> position;
    position[0] = data.m_localCollisionFrame[0];
    position[1] = data.m_localCollisionFrame[1];
    position[2] = data.m_localCollisionFrame[2];

    switch (data.m_collisionGeometryType) {
      case GEOM_BOX:
      {
        benchmark::Vec<4> dim = {
            data.m_dimensions[0],
            data.m_dimensions[1],
            data.m_dimensions[2],
            0
        };
        visColObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Box));
        visColProps_.emplace_back(std::make_pair("", dim));
        break;
      }
      case GEOM_CAPSULE:
      {
        benchmark::Vec<4> dim = {
            data.m_dimensions[1],   // radius
            data.m_dimensions[0],   // length
            0
        };
        visColObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Capsule));
        visColProps_.emplace_back(std::make_pair("", dim));
        break;
      }
      case GEOM_CYLINDER:
      {
        benchmark::Vec<4> dim = {
            data.m_dimensions[1],   // radius
            data.m_dimensions[0],   // length
            0,
            0
        };
        visColObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Cylinder));
        visColProps_.emplace_back(std::make_pair("", dim));
        break;
      }
      case GEOM_SPHERE:
      {
        benchmark::Vec<4> dim = {
            data.m_dimensions[0],   // radius
            0,
            0,
            0
        };
        visColObj.emplace_back(std::make_tuple(mat, position, linkId, benchmark::object::Shape::Sphere));
        visColProps_.emplace_back(std::make_pair("", dim));
        break;
      }
      default:
      RAIFATAL("not supported collision shape")
    }
  }
}

int object::BtMbArticulatedSystem::getDOF() {
  return dof_;
}

int object::BtMbArticulatedSystem::getStateDimension() {
  return stateDimension_;
}

const benchmark::object::ArticulatedSystemInterface::EigenVec object::BtMbArticulatedSystem::getGeneralizedCoordinate() {
  if(isFixed_) {
    // joints
    for(int i = 0; i < numJoints_; i++) {
      b3JointSensorState state;
      RAIFATAL_IF(!api_->getJointState(objectId_, ctrbJoints_[i], &state), "getJointState failed");
      genCoordinate_[i] = state.m_jointPosition;
    }
  } // end of fixed base
  else {
    btVector3 bPosition;
    btQuaternion bQuaternion;
    RAIFATAL_IF(!api_->getBasePositionAndOrientation(objectId_, bPosition, bQuaternion),
                "getBasePositionAndOrientation failed");

    {
      // base
      genCoordinate_[0] = bPosition.x();
      genCoordinate_[1] = bPosition.y();
      genCoordinate_[2] = bPosition.z();

      genCoordinate_[3] = bQuaternion.w();
      genCoordinate_[4] = bQuaternion.x();
      genCoordinate_[5] = bQuaternion.y();
      genCoordinate_[6] = bQuaternion.z();
    }

    // joints
    for(int i = 0; i < numJoints_; i++) {
      b3JointSensorState state;
      RAIFATAL_IF(!api_->getJointState(objectId_, ctrbJoints_[i], &state), "getJointState failed");
      genCoordinate_[i+7] = state.m_jointPosition;
    }
  } // end of floating base

  return genCoordinate_.e();
}

const benchmark::object::ArticulatedSystemInterface::EigenVec object::BtMbArticulatedSystem::getGeneralizedVelocity() {

  if(isFixed_) {
    // joints
    for(int i = 0; i < numJoints_; i++) {
      b3JointSensorState state;
      RAIFATAL_IF(!api_->getJointState(objectId_, ctrbJoints_[i], &state), "getJointState failed");
      genVelocity_[i] = state.m_jointVelocity;
    }
  } // end of fixed base
  else {
    btVector3 bLinVel;
    btVector3 bAngVel;
    RAIFATAL_IF(!api_->getBaseVelocity(objectId_, bLinVel, bAngVel),
                "getBaseVelocity failed")

    {
      // base
      genVelocity_[0] = bLinVel.x();
      genVelocity_[1] = bLinVel.y();
      genVelocity_[2] = bLinVel.z();

      genVelocity_[3] = bAngVel.x();
      genVelocity_[4] = bAngVel.y();
      genVelocity_[5] = bAngVel.z();
    }

    // joints
    for(int i = 0; i < numJoints_; i++) {
      b3JointSensorState state;
      RAIFATAL_IF(!api_->getJointState(objectId_, ctrbJoints_[i], &state), "getJointState failed");
      genVelocity_[i+6] = state.m_jointVelocity;
    }
  } // end of floating base
  return genVelocity_.e();
}

const benchmark::object::ArticulatedSystemInterface::EigenVec object::BtMbArticulatedSystem::getGeneralizedForce() {
  return genForce_.e();
}

void object::BtMbArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  genco = getGeneralizedCoordinate();
  genvel = getGeneralizedVelocity();
}

void object::BtMbArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  RAIFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input");

  if(isFixed_) {
    for(int i = 0; i < numJoints_; i++) {
      genCoordinate_[i] = jointState[i];
      api_->resetJointState(objectId_, ctrbJoints_[i], jointState[i]);
    }
  } // end of fixed base
  else {

    // floating base
    {
      // base
      btVector3 basePosition = {
          jointState[0],
          jointState[1],
          jointState[2]
      };

      genCoordinate_[0] = basePosition.x();
      genCoordinate_[1] = basePosition.y();
      genCoordinate_[2] = basePosition.z();

      btQuaternion baseQuaternion = {
          jointState[4], // x
          jointState[5], // y
          jointState[6], // z
          jointState[3], // w
      };

      genCoordinate_[3] = baseQuaternion.w();
      genCoordinate_[4] = baseQuaternion.x();
      genCoordinate_[5] = baseQuaternion.y();
      genCoordinate_[6] = baseQuaternion.z();

      api_->resetBasePositionAndOrientation(objectId_, basePosition, baseQuaternion);
    }

    // joint
    for(int i = 0; i < numJoints_; i++) {
      genCoordinate_[i+7] = jointState[i+7];
      api_->resetJointState(objectId_, ctrbJoints_[i], jointState[i+7]);
    }

  } // end of floating base
}

void object::BtMbArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  RAIFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input");

  if(isFixed_) {
    // fixed base
    for(int i = 0; i < numJoints_; i++) {
      genCoordinate_[i] = jointState.begin()[i];
      api_->resetJointState(objectId_, ctrbJoints_[i], jointState.begin()[i]);
    }
  } // end of fixed base
  else {

    // floating base
    {
      // base
      btVector3 basePosition = {
          jointState.begin()[0],
          jointState.begin()[1],
          jointState.begin()[2]
      };

      genCoordinate_[0] = basePosition.x();
      genCoordinate_[1] = basePosition.y();
      genCoordinate_[2] = basePosition.z();

      btQuaternion baseQuaternion = {
          jointState.begin()[4], // x
          jointState.begin()[5], // y
          jointState.begin()[6], // z
          jointState.begin()[3], // w
      };

      genCoordinate_[3] = baseQuaternion.w();
      genCoordinate_[4] = baseQuaternion.x();
      genCoordinate_[5] = baseQuaternion.y();
      genCoordinate_[6] = baseQuaternion.z();

      api_->resetBasePositionAndOrientation(objectId_, basePosition, baseQuaternion);
    }

    // joint
    for(int i = 0; i < numJoints_; i++) {
      genCoordinate_[i+7] = jointState.begin()[i+7];
      api_->resetJointState(objectId_, ctrbJoints_[i], jointState.begin()[i+7]);
    }
  } // floating base
}

void object::BtMbArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  RAIWARN("direct assigning joint velocity is not available. set only base velocity")
  if(isFixed_){

  }
  else {
    btVector3 bLinVel = {
        jointVel[0],
        jointVel[1],
        jointVel[2]
    };
    btVector3 bAngVel = {
        jointVel[3],
        jointVel[4],
        jointVel[5]
    };

    RAIFATAL_IF(!api_->resetBaseVelocity(objectId_, bLinVel, bAngVel), "resetBaseVelocity failed");
  }
}

void object::BtMbArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
  RAIWARN("direct assigning joint velocity is not available. set only base velocity")
  if(isFixed_){

  }
  else {
    btVector3 bLinVel = {
        jointVel.begin()[0],
        jointVel.begin()[1],
        jointVel.begin()[2]
    };
    btVector3 bAngVel = {
        jointVel.begin()[3],
        jointVel.begin()[4],
        jointVel.begin()[5]
    };

    RAIFATAL_IF(!api_->resetBaseVelocity(objectId_, bLinVel, bAngVel), "resetBaseVelocity failed");
  }
}

void object::BtMbArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")

  if(isFixed_) {
    // fixed base
    // joint
    {
      b3RobotSimulatorJointMotorArrayArgs arg(CONTROL_MODE_TORQUE, numJoints_);

      double jointForces[numJoints_];
      for(int i = 0; i < numJoints_; i++) {
        jointForces[i] = tau[i];
        genForce_[i] = tau[i];
      }
      arg.m_forces = jointForces;
      arg.m_jointIndices = &ctrbJoints_[0]; // vector -> array conversion
      arg.m_targetPositions = 0;
      arg.m_targetVelocities = 0;

      RAIFATAL_IF(!api_->setJointMotorControlArray(objectId_, arg), "setJointMotorControlArray failed")
    }
  } // end of fixed base
  else {

    // floating base
    {
      // base
      btVector3 bForce = {
          tau[0],
          tau[1],
          tau[2]
      };

      genForce_[0] = tau[0];
      genForce_[1] = tau[1];
      genForce_[2] = tau[2];

      btVector3 bTorque = {
          tau[3], // x
          tau[4], // y
          tau[5], // z
      };

      genForce_[3] = tau[3];
      genForce_[4] = tau[4];
      genForce_[5] = tau[5];

      btVector3 bPosition = {0, 0, 0};
      api_->applyExternalForce(objectId_, -1, bForce, bPosition, EF_WORLD_FRAME);
      api_->applyExternalTorque(objectId_, -1, bTorque, EF_WORLD_FRAME);
    }

    // joint
    {
      b3RobotSimulatorJointMotorArrayArgs arg(CONTROL_MODE_TORQUE, numJoints_);

      double jointForces[numJoints_];
      for(int i = 0; i < numJoints_; i++) {
        jointForces[i] = tau[i+6];
        genForce_[i+6] = tau[i+6];
      }
      arg.m_forces = jointForces;
      arg.m_jointIndices = &ctrbJoints_[0]; // vector -> array conversion
      arg.m_targetPositions = 0;
      arg.m_targetVelocities = 0;

      RAIFATAL_IF(!api_->setJointMotorControlArray(objectId_, arg), "setJointMotorControlArray failed")
    }

  } // floating base
}

void object::BtMbArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")

  if(isFixed_) {
    // fixed base
    // joint
    {
      b3RobotSimulatorJointMotorArrayArgs arg(CONTROL_MODE_TORQUE, numJoints_);

      double jointForces[numJoints_];
      for(int i = 0; i < numJoints_; i++) {
        jointForces[i] = tau.begin()[i];
        genForce_[i] = tau.begin()[i];
      }
      arg.m_forces = jointForces;
      arg.m_jointIndices = &ctrbJoints_[0]; // vector -> array conversion

      RAIFATAL_IF(!api_->setJointMotorControlArray(objectId_, arg), "setJointMotorControlArray failed")
    }
  } // end of fixed base
  else {

    // floating base
    {
      // base
      btVector3 bForce = {
          tau.begin()[0],
          tau.begin()[1],
          tau.begin()[2]
      };

      genForce_[0] = tau.begin()[0];
      genForce_[1] = tau.begin()[1];
      genForce_[2] = tau.begin()[2];

      btVector3 bTorque = {
          tau.begin()[3], // x
          tau.begin()[4], // y
          tau.begin()[5], // z
      };

      genForce_[3] = tau.begin()[3];
      genForce_[4] = tau.begin()[4];
      genForce_[5] = tau.begin()[5];

      btVector3 bPosition = {0, 0, 0};
      api_->applyExternalForce(objectId_, -1, bForce, bPosition, EF_WORLD_FRAME);
      api_->applyExternalTorque(objectId_, -1, bTorque, EF_WORLD_FRAME);
    }

    // joint
    {
      b3RobotSimulatorJointMotorArrayArgs arg(CONTROL_MODE_TORQUE, numJoints_);

      double jointForces[numJoints_];
      for(int i = 0; i < numJoints_; i++) {
        jointForces[i] = tau.begin()[i+6];
        genForce_[i+6] = tau.begin()[i+6];
      }
      arg.m_forces = jointForces;
      arg.m_jointIndices = &ctrbJoints_[0]; // vector -> array conversion

      RAIFATAL_IF(!api_->setJointMotorControlArray(objectId_, arg), "setJointMotorControlArray failed")
    }

  } // floating base
}

void object::BtMbArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  setGeneralizedCoordinate(genco);
  setGeneralizedVelocity(genvel);
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> object::BtMbArticulatedSystem::getLinearMomentumInCartesianSpace() {

  {
    // base
    b3LinkState state;
    api_->getLinkState(objectId_, -1, &state);

    linearMomentum_[0] = state.m_worldLinearVelocity[0] * mass_[0];
    linearMomentum_[1] = state.m_worldLinearVelocity[1] * mass_[0];
    linearMomentum_[2] = state.m_worldLinearVelocity[2] * mass_[0];
  }

  // link
  for (int i = 1; i < mass_.size(); i++) {
    b3LinkState state;
    api_->getLinkState(objectId_, i, &state);

    linearMomentum_[0] += state.m_worldLinearVelocity[0] * mass_[i];
    linearMomentum_[1] += state.m_worldLinearVelocity[1] * mass_[i];
    linearMomentum_[2] += state.m_worldLinearVelocity[2] * mass_[i];
  }

  return linearMomentum_.e();
}

double object::BtMbArticulatedSystem::getTotalMass() {
  double mass = 0;
  for(double linkmass : mass_)
    mass += linkmass;
  return mass;
}

double object::BtMbArticulatedSystem::getKineticEnergy() {
  double kinetic = 0;

  {
    // base
    // kinetic energy (x 0.5 at the last)
    btVector3 bLinVel;
    btVector3 bAngvel;
    api_->getBaseVelocity(objectId_, bLinVel, bAngvel);

    btVector3 bPosition;
    btQuaternion bQuaternion;
    api_->getBasePositionAndOrientation(objectId_, bPosition, bQuaternion);

    kinetic += mass_[0] * bLinVel.dot(bLinVel);

    bAngvel = btMatrix3x3(bQuaternion) * bAngvel;   // get local angvel
    benchmark::Vec<3> angVel = {
        bAngvel.x(),
        bAngvel.y(),
        bAngvel.z()
    };

    double angular = 0;
    benchmark::vecTransposeMatVecMul(angVel, inertia_[0], angular);
    kinetic += angular;
  }

  {
    // link
    for (int i = 1; i < mass_.size(); i++) {
      b3LinkState state;
      api_->getLinkState(objectId_, i, &state);
      {
        kinetic += mass_[i] * (
            pow(state.m_worldLinearVelocity[0], 2)
                + pow(state.m_worldLinearVelocity[1], 2)
                + pow(state.m_worldLinearVelocity[2], 2)
        );

        double angular;
        benchmark::Vec<3> angVel;
        benchmark::vecTransposeMatVecMul(angVel, inertia_[i], angular);
        kinetic += angular;
      }

    }
  }

  return 0.5 * kinetic;
}

double object::BtMbArticulatedSystem::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
  double potential = 0;

  {
    // base
    btVector3 bPosition;
    btQuaternion bQuaternion;
    api_->getBasePositionAndOrientation(objectId_, bPosition, bQuaternion);

    // potential energy
    potential -= mass_[0] * bPosition.dot({gravity[0], gravity[1], gravity[2]});
  }

  {
    // link
    for (int i = 1; i < mass_.size(); i++) {
      b3LinkState state;
      api_->getLinkState(objectId_, i, &state);

      {
        btVector3 compos(
            state.m_worldPosition[0],
            state.m_worldPosition[1],
            state.m_worldPosition[2]
        );
        potential -= mass_[i] * compos.dot({gravity[0], gravity[1], gravity[2]});
      }
    }
  }

  return potential;
}

double object::BtMbArticulatedSystem::getEnergy(const benchmark::Vec<3> &gravity) {
  return getKineticEnergy() + getPotentialEnergy(gravity);
}

void object::BtMbArticulatedSystem::getBodyPose(int linkId,
                                                benchmark::Mat<3, 3> &orientation,
                                                benchmark::Vec<3> &position) {

  if(linkId == -1) {
    // base
    btVector3 bPosition;
    btQuaternion bQuat;
    RAIFATAL_IF(!api_->getBasePositionAndOrientation(objectId_, bPosition, bQuat), "getBasePositionAndOrientation failed");
    position = {
        bPosition.x(),
        bPosition.y(),
        bPosition.z()
    };

    benchmark::Vec<4> quat = {
        bQuat.w(),
        bQuat.x(),
        bQuat.y(),
        bQuat.z(),
    };
    benchmark::quatToRotMat(quat, orientation);
  }
  else {
    // link
    b3LinkState linkState;
    RAIFATAL_IF(!api_->getLinkState(objectId_, linkId, &linkState), "getBasePositionAndOrientation failed");

    btTransform comTf_W;
    comTf_W.setOrigin(
        {linkState.m_worldPosition[0],
         linkState.m_worldPosition[1],
         linkState.m_worldPosition[2]});
    comTf_W.setRotation(
        {linkState.m_worldOrientation[0],
         linkState.m_worldOrientation[1],
         linkState.m_worldOrientation[2],
         linkState.m_worldOrientation[3]});

    btTransform comTf_B;
    comTf_B.setOrigin(
        {linkState.m_localInertialPosition[0],
         linkState.m_localInertialPosition[1],
         linkState.m_localInertialPosition[2]});
    comTf_B.setRotation(
        {linkState.m_localInertialOrientation[0],
         linkState.m_localInertialOrientation[1],
         linkState.m_localInertialOrientation[2],
         linkState.m_localInertialOrientation[3]});

    btTransform bTf = comTf_W * comTf_B.inverse();

    benchmark::Vec<4> quat = {
        bTf.getRotation().w(),
        bTf.getRotation().x(),
        bTf.getRotation().y(),
        bTf.getRotation().z()};
    benchmark::quatToRotMat(quat, orientation);

    position = {
        bTf.getOrigin().x(),
        bTf.getOrigin().y(),
        bTf.getOrigin().z()};
  }
}

} // bullet_mb_sim