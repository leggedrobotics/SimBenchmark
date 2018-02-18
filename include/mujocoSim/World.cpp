//
// Created by kangd on 18.02.18.
//

#include "World.hpp"
#include <raiCommon/utils/rai_message_logger/rai_message.hpp>

mujoco_sim::World::World(const char* modelPath) {

  // activate MuJoCo Pro
  mj_activate("mjkey.txt");

  // load model
  char error[1000];

  worldModel_ = mj_loadXML(modelPath, NULL, error, 1000);
  if( !worldModel_ )
  {
    RAIFATAL(error);
  }

  // make data corresponding to model
  worldData_ = mj_makeData(worldModel_);

}

mujoco_sim::World::~World() {

  // free model and data, deactivate
  mj_deleteData(worldData_);
  mj_deleteModel(worldModel_);
  mj_deactivate();

}

mjModel *mujoco_sim::World::getWorldModel() const {
  return worldModel_;
}
mjData *mujoco_sim::World::getWorldData() const {
  return worldData_;
}


