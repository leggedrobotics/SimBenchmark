//
// Created by kangd on 24.05.18.
//

#include "BtMbWorld.hpp"

namespace bullet_mb_sim {

BtMbWorld::BtMbWorld() {
  api_ = new b3RobotSimulatorClientAPI_NoGUI();

  bool isConnected = api_->connect(eCONNECT_SHARED_MEMORY);

  if (!isConnected) {
    RAIINFO("BtMbSim initializing: Using Direct mode...");
    isConnected = api_->connect(eCONNECT_DIRECT);
  }
  else
  {
    RAIINFO("BtMbSim initializing: Using shared memory...");
  }

  //remove all existing objects (if any)
  api_->resetSimulation();

  {
    // engine parameters
    b3RobotSimulatorSetPhysicsEngineParameters arg;
    arg.m_defaultContactERP = 0;
    arg.m_defaultNonContactERP = 0;
    arg.m_frictionERP = 0;
//    arg.m_solverResidualThreshold = 1e-4;       // TODO tuning!
//    arg.m_restitutionVelocityThreshold = 0;     // TODO tuning!
//    arg.m_defaultGlobalCFM = 0;
//    arg.m_frictionCFM = 0;
    api_->setPhysicsEngineParameter(arg);
  }
}

BtMbWorld::~BtMbWorld() {
  delete api_;

  for(auto *ob: objectList_)
    delete ob;
}

void BtMbWorld::setGravity(const benchmark::Vec<3> &gravity) {
  api_->setGravity({gravity[0],
                    gravity[1],
                    gravity[2]});
}

int BtMbWorld::getNumObject() {
  return api_->getNumBodies();
}

object::BtMbArticulatedSystem * BtMbWorld::addArticulatedSystem(std::string nm,
                                                                object::ObjectFileType fileType,
                                                                bool internalCollision,
                                                                bool maximalCoordinate,
                                                                benchmark::CollisionGroupType collisionGroup,
                                                                benchmark::CollisionGroupType collisionMask) {
  auto *articulatedSystem =
      new object::BtMbArticulatedSystem(nm, fileType, internalCollision, maximalCoordinate, api_);
  objectList_.push_back(articulatedSystem);
  return articulatedSystem;
}

benchmark::object::SingleBodyObjectInterface *BtMbWorld::addCheckerboard(double gridSize,
                                                                         double xLength,
                                                                         double yLength,
                                                                         double reflectanceI,
                                                                         bo::CheckerboardShape shape,
                                                                         benchmark::CollisionGroupType collisionGroup,
                                                                         benchmark::CollisionGroupType collisionMask) {
  auto *checkerBoard = new bullet_mb_sim::object::BtMbCheckerBoard(xLength, yLength, api_, shape);
  objectList_.push_back(checkerBoard);
  return checkerBoard;
}

benchmark::object::SingleBodyObjectInterface *BtMbWorld::addSphere(double radius,
                                                                   double mass,
                                                                   benchmark::CollisionGroupType collisionGroup,
                                                                   benchmark::CollisionGroupType collisionMask) {
  auto *sphere = new bullet_mb_sim::object::BtMbSphere(radius, mass, api_);
  objectList_.push_back(sphere);
  return sphere;
}

benchmark::object::SingleBodyObjectInterface *BtMbWorld::addBox(double xLength,
                                                                double yLength,
                                                                double zLength,
                                                                double mass,
                                                                benchmark::CollisionGroupType collisionGroup,
                                                                benchmark::CollisionGroupType collisionMask) {
  auto *box = new bullet_mb_sim::object::BtMbBox(xLength, yLength, zLength, mass, api_);
  objectList_.push_back(box);
  return box;
}

void BtMbWorld::integrate() {
  // collision detection
  contactProblemList_.clear();
  b3RobotSimulatorGetContactPointsArgs arg;
  b3ContactInformation info;
  api_->getContactPoints(arg, &info);
  contactProblemList_.reserve(info.m_numContactPoints);
  
  for (int i = 0; i < info.m_numContactPoints; ++i) {
    btVector3 position(
        info.m_contactPointData[i].m_positionOnAInWS[0],
        info.m_contactPointData[i].m_positionOnAInWS[1],
        info.m_contactPointData[i].m_positionOnAInWS[2]
    );

    btVector3 normal(
        info.m_contactPointData[i].m_contactNormalOnBInWS[0],
        info.m_contactPointData[i].m_contactNormalOnBInWS[1],
        info.m_contactPointData[i].m_contactNormalOnBInWS[2]
    );

    contactProblemList_.emplace_back(position, normal);
  }

  // step simulation
  api_->stepSimulation();
}

const std::vector<Single3DContactProblem> *BtMbWorld::getCollisionProblem() const {
  return &contactProblemList_;
}

const std::vector<object::BtMbObject *> &BtMbWorld::getObjectList() const {
  return objectList_;
}

} // bullet_mb_sim
