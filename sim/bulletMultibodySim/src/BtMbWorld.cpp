//
// Created by kangd on 24.05.18.
//

#include "BtMbWorld.hpp"

namespace bullet_multibody_sim {

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
}

BtMbWorld::~BtMbWorld() {
  delete api_;
}

void BtMbWorld::setGravity(const benchmark::Vec<3> &gravity) {
  api_->setGravity({float(gravity[0]),
                    float(gravity[1]),
                    float(gravity[2])});
}

int BtMbWorld::getNumObject() {
  return api_->getNumBodies();
}

object::BtMbArticulatedSystem* BtMbWorld::addArticulatedSystem(std::string nm,
                                                                object::ObjectFileType fileType,
                                                                benchmark::CollisionGroupType collisionGroup,
                                                                benchmark::CollisionGroupType collisionMask) {

  object::BtMbArticulatedSystem *articulatedSystem =
      new object::BtMbArticulatedSystem(nm, fileType, api_);
  objectList_.push_back(articulatedSystem);
  return articulatedSystem;
}

} // bullet_multibody_sim
