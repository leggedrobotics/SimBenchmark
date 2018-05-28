//
// Created by kangd on 27.05.18.
//

#include <common/interface/CheckerboardInterface.hpp>
#include "BtMbCheckerBoard.hpp"
#include "BtMbSingleBodyObject.hpp"

namespace bullet_mb_sim {
namespace object {

BtMbCheckerBoard::BtMbCheckerBoard(double xLength,
                                   double yLength,
                                   b3RobotSimulatorClientAPI_NoGUI *api,
                                   bo::CheckerboardShape shape)
    : BtMbSingleBodyObject(0, api) {

  // create collision shape
  int shapeId = -1;
  {
    if (shape == bo::PLANE_SHAPE) {
      b3RobotSimulatorCreateCollisionShapeArgs arg;
      arg.m_shapeType = GEOM_PLANE;
      arg.m_planeNormal = {0, 0, 1};
      shapeId = api_->createCollisionShape(GEOM_PLANE, arg);
    } else if (shape == bo::BOX_SHAPE) {
      b3RobotSimulatorCreateCollisionShapeArgs arg;
      arg.m_shapeType = GEOM_BOX;
      arg.m_halfExtents = {xLength * 0.5, yLength * 0.5, 10.0};
      shapeId = api_->createCollisionShape(GEOM_PLANE, arg);
    } else {
      RAIFATAL("invalid shape input")
    }
    RAIFATAL_IF(shapeId ==  -1, "Checkerboard shape creation error")
  }

  // create body
  {
    b3RobotSimulatorCreateMultiBodyArgs args;
    args.m_baseCollisionShapeIndex = shapeId;
    args.m_useMaximalCoordinates = 1;
    objectId_ = api_->createMultiBody(args);

    b3RobotSimulatorChangeDynamicsArgs dynarg;
    dynarg.m_lateralFriction = 0.8;
    dynarg.m_restitution = 0;
    dynarg.m_rollingFriction = 0;
    dynarg.m_spinningFriction = 0;
    RAIFATAL_IF(!api_->changeDynamics(objectId_, -1, dynarg), "changeDynamics failed")
  }

  RAIFATAL_IF(objectId_ ==  -1, "Checkerboard body creation error")
}

} // object
} // bullet_mb_sim