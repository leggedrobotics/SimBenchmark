//
// Created by kangd on 18.02.18.
//

#include "World.hpp"

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

  // make objects
  for(int i = 0; i < worldModel_->ngeom; i++) {

    switch (*(worldModel_->geom_type + i)) {
      case mjGEOM_PLANE:
      case mjGEOM_SPHERE:
      case mjGEOM_CAPSULE:
      case mjGEOM_BOX:
        break;
      default:
        RAIFATAL("wrong geometry type");
    }

    object::SingleBodyObject *object = new object::SingleBodyObject(worldData_, i);
    objectList_.push_back(object);
  }
}

mujoco_sim::World::~World() {

  // remove objects
  for (auto *ob: objectList_)
    delete ob;

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
const std::vector<mujoco_sim::object::SingleBodyObject *> &mujoco_sim::World::getObjectList() const {
  return objectList_;
}


