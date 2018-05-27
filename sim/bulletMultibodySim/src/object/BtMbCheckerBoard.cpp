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
    : BtMbSingleBodyObject(api) {

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
  }

  RAIFATAL_IF(objectId_ ==  -1, "Checkerboard body creation error")
  mass_ = 0;
}

} // object
} // bullet_mb_sim