//
// Created by kangd on 28.05.18.
//

#include "BtMbBox.hpp"

namespace bullet_mb_sim {
namespace object {

bullet_mb_sim::object::BtMbBox::BtMbBox(double xlength,
                                        double ylength,
                                        double zlength,
                                        double mass,
                                        b3RobotSimulatorClientAPI_NoGUI *api)
    : BtMbSingleBodyObject(mass, api) {

  // create collision shape
  int shapeId = -1;
  {
    b3RobotSimulatorCreateCollisionShapeArgs arg;
    arg.m_shapeType = GEOM_BOX;
    arg.m_halfExtents = btVector3(
        xlength * 0.5,
        ylength * 0.5,
        zlength * 0.5
    );
    shapeId = api_->createCollisionShape(GEOM_BOX, arg);
    RAIFATAL_IF(shapeId ==  -1, "Checkerboard shape creation error")
  }

  // create body
  {
    b3RobotSimulatorCreateMultiBodyArgs args;
    args.m_baseCollisionShapeIndex = shapeId;
    args.m_useMaximalCoordinates = 1;
    args.m_baseMass = mass;
    objectId_ = api_->createMultiBody(args);

    b3RobotSimulatorChangeDynamicsArgs dynarg;
    dynarg.m_lateralFriction = 0.8;
    dynarg.m_restitution = 0;
    dynarg.m_rollingFriction = 0;
    dynarg.m_spinningFriction = 0;
    RAIFATAL_IF(!api_->changeDynamics(objectId_, -1, dynarg), "changeDynamics failed")
  }

  // inertia
  {
    b3DynamicsInfo info;
    RAIFATAL_IF(!api_->getDynamicsInfo(objectId_, -1, &info), "getDynamicsInfo failed")
    localInertia_[0] = info.m_localInertialDiagonal[0];
    localInertia_[4] = info.m_localInertialDiagonal[1];
    localInertia_[8] = info.m_localInertialDiagonal[2];
  }
  RAIFATAL_IF(objectId_ ==  -1, "Checkerboard body creation error")
}

} // object
} // bullet_multi_body
