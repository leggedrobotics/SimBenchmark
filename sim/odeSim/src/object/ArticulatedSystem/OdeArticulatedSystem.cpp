//
// Created by kangd on 15.04.18.
//

#include "OdeArticulatedSystem.hpp"

namespace ode_sim {
namespace object {

OdeArticulatedSystem::OdeArticulatedSystem(std::string urdfFile,
                                           const dWorldID worldID,
                                           const dSpaceID spaceID,
                                           int collisionGroup,
                                           int collisionMask)
    : worldID_(worldID), spaceID_(spaceID), collisionGroup_(collisionGroup), collisionMask_(collisionMask) {

  urdfFile += "robot.urdf";
  std::ifstream model_file(urdfFile);
  RAIFATAL_IF(!model_file.good(), "Error opening file: " << urdfFile);

  // reserve memory for the contents of the file
  std::string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());
  model_file.close();

  unsigned long searchPosition = 0;
  jointsNames_ = std::vector<std::string>();

  // find all joint names
  while (true) {
    unsigned long position = model_xml_string.find("joint name", searchPosition);
    if (position == std::string::npos) break;
    unsigned long nameStart = model_xml_string.find("\"", position);
    unsigned long nameEnd = model_xml_string.find("\"", nameStart + 1);

    if (std::find(jointsNames_.begin(), jointsNames_.end(), model_xml_string.substr(nameStart + 1, nameEnd - nameStart - 1)) == jointsNames_.end())
      jointsNames_.push_back(model_xml_string.substr(nameStart + 1, nameEnd - nameStart - 1));
    searchPosition = nameEnd;
  }

  boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDFFile(urdfFile);
  boost::shared_ptr<const urdf::Link> rootLink = urdf_model->getRoot();

  if (rootLink->name != "world") {
    // float base
    isFixed_ = false;
    dof_ = 6;
    stateDimension_ = 7;
  } else {
    // fixed base
    isFixed_ = true;
    dof_ = 0;
    stateDimension_ = 0;

    if (rootLink->child_links[0]->parent_joint->type == urdf::Joint::FIXED)
      rootLink = rootLink->child_links[0];
  }

  // rootlink's parent is world
  rootLink_.parentIdx_ = -1;

  // process links recursively
  processLinkFromUrdf(rootLink, rootLink_, jointsNames_);

  // init articulated system
  init();
}

void OdeArticulatedSystem::processLinkFromUrdf(boost::shared_ptr<const urdf::Link> urdfLink,
                                               Link &raiLink,
                                               std::vector<std::string> &jointsOrder) {

  // link init
  raiLink.name_ = urdfLink->name;
  raiLink.odeBody_ = dBodyCreate(worldID_);

  /// parent
  if(urdfLink->getParent()) {
    raiLink.parentJointName_ = urdfLink->parent_joint->name;
    raiLink.parentName_ = urdfLink->getParent()->name;
  } else {
    raiLink.parentJointName_ = "";
    raiLink.parentName_ = "";
  }

  /// inertial
  if (urdfLink->inertial) {
    raiLink.inertial_.mass_ = urdfLink->inertial->mass;
    raiLink.inertial_.inertia_.e() << urdfLink->inertial->ixx, urdfLink->inertial->ixy, urdfLink->inertial->ixz,
        urdfLink->inertial->ixy, urdfLink->inertial->iyy, urdfLink->inertial->iyz,
        urdfLink->inertial->ixz, urdfLink->inertial->ixy, urdfLink->inertial->izz;

    raiLink.inertial_.pos_.e() << urdfLink->inertial->origin.position.x,
        urdfLink->inertial->origin.position.y,
        urdfLink->inertial->origin.position.z;

    benchmark::Vec<3> rpy;
    urdfLink->inertial->origin.rotation.getRPY(rpy[0], rpy[1], rpy[2]);
    benchmark::rpyToRotMat_intrinsic(rpy, raiLink.inertial_.rotmat_);
  }
  else {
    raiLink.inertial_.mass_ = 0;
    raiLink.inertial_.inertia_.setZero();
    raiLink.inertial_.pos_.setZero();
    raiLink.inertial_.rotmat_.setIdentity();
  }

  /// collision
  if (urdfLink->collision) {
    // collision object
    for (auto &col: urdfLink->collision_array) {
      if (col->geometry->type == urdf::Geometry::BOX) {
        // box
        auto box = boost::dynamic_pointer_cast<urdf::Box>(col->geometry);
        raiLink.collision_.colShape_.push_back(bo::Shape::Box);
        raiLink.collision_.colShapeParam_.push_back({box->dim.x, box->dim.y, box->dim.z, 0});
      }
      else if (col->geometry->type == urdf::Geometry::CYLINDER) {
        // cylinder
        auto cyl = boost::dynamic_pointer_cast<urdf::Cylinder>(col->geometry);
        raiLink.collision_.colShape_.push_back(bo::Shape::Cylinder);
        raiLink.collision_.colShapeParam_.push_back({cyl->radius, cyl->length, 0, 0});
      }
      else if (col->geometry->type == urdf::Geometry::SPHERE) {
        // sphere
        auto sph = boost::dynamic_pointer_cast<urdf::Sphere>(col->geometry);
        raiLink.collision_.colShape_.push_back(bo::Shape::Sphere);
        raiLink.collision_.colShapeParam_.push_back({sph->radius, 0, 0, 0});
      }
      else {
        RAIFATAL("mesh collision body is not supported yet");
      }

      // relative position w.r.t. parent joint
      auto &pos = col->origin.position;
      double r, p, y;
      col->origin.rotation.getRPY(r, p, y);
      raiLink.collision_.colObjOrigin_.push_back({pos.x, pos.y, pos.z});
      raiLink.collision_.colOrientation({r, p, y});
    }
  }

  /// visual
  if (urdfLink->visual) {
    for (auto &vis: urdfLink->visual_array) {
      if (vis->geometry->type == urdf::Geometry::BOX) {
        // box
        auto box = boost::dynamic_pointer_cast<urdf::Box>(vis->geometry);
        raiLink.visual_.visshape_.push_back(bo::Shape::Box);
        raiLink.visual_.visShapeParam_.push_back({box->dim.x, box->dim.y, box->dim.z, 0});
        raiLink.visual_.meshFileNames_.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::CYLINDER) {
        // cylinder
        auto cyl = boost::dynamic_pointer_cast<urdf::Cylinder>(vis->geometry);
        raiLink.visual_.visshape_.push_back(bo::Shape::Cylinder);
        raiLink.visual_.visShapeParam_.push_back({cyl->radius, cyl->length, 0, 0});
        raiLink.visual_.meshFileNames_.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::SPHERE) {
        // sphere
        auto sph = boost::dynamic_pointer_cast<urdf::Sphere>(vis->geometry);
        raiLink.visual_.visshape_.push_back(bo::Shape::Sphere);
        raiLink.visual_.visShapeParam_.push_back({sph->radius, 0, 0, 0});
        raiLink.visual_.meshFileNames_.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::MESH) {
        // mesh
        auto mes = boost::dynamic_pointer_cast<urdf::Mesh>(vis->geometry);
        raiLink.visual_.visshape_.push_back(bo::Shape::Mesh);
        raiLink.visual_.visShapeParam_.push_back({mes->scale.x, mes->scale.y, mes->scale.z, 0});
        raiLink.visual_.meshFileNames_.push_back(mes->filename);
      }

      if (vis->material)
        raiLink.visual_.visColor_.push_back(
            {vis->material->color.r,
             vis->material->color.g,
             vis->material->color.b,
             vis->material->color.a});
      else
        raiLink.visual_.visColor_.push_back(
            {0.7, 0.7, 0.7, 1.0});

      auto &pos = vis->origin.position;
      double r, p, y;
      vis->origin.rotation.getRPY(r, p, y);
      raiLink.visual_.visObjOrigin_.push_back({pos.x, pos.y, pos.z});
      raiLink.visual_.visOrientation({r, p, y});
    }
  }

  /// urdfreader order joints alphabetically. We follow the order defined in the urdf
  std::vector<int> childOrder;

  for (auto &name : jointsOrder)
    for (int i = 0; i < urdfLink->child_links.size(); i++)
      if (urdfLink->child_links[i]->parent_joint->name == name)
        childOrder.push_back(i);

  int jointsNumber = childOrder.size();

  /// children links
  for(int i = 0; i < urdfLink->child_links.size(); i++) {
    Link *childRef;
    auto ch = urdfLink->child_links[childOrder[i]];
    auto &jnt = ch->parent_joint;
//    childRef->parentJoint_.jointName_ = jnt->name;

    switch (jnt->type) {
      case urdf::Joint::FIXED: {
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::FIXED;
        break;
      }
      case urdf::Joint::REVOLUTE: {
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::REVOLUTE;
        break;
      }
      case urdf::Joint::PRISMATIC: {
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::PRISMATIC;
        break;
      }
      default:
      RAIFATAL("currently only support revolute/prismatic/fixed joint");
    }

    double r, p, y;
    jnt->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
    benchmark::Vec<3> rpy;
    rpy.e() << r, p, y;
    double axisNorm = sqrt(jnt->axis.x * jnt->axis.x + jnt->axis.y * jnt->axis.y + jnt->axis.z * jnt->axis.z);
    childRef->parentJoint_.jointAxis({jnt->axis.x / axisNorm, jnt->axis.y / axisNorm, jnt->axis.z / axisNorm});
    childRef->parentJoint_.jointPosition({jnt->parent_to_joint_origin_transform.position.x,
                                          jnt->parent_to_joint_origin_transform.position.y,
                                          jnt->parent_to_joint_origin_transform.position.z});
    benchmark::rpyToRotMat_intrinsic(rpy, childRef->parentJoint_.rotmat_);

    processLinkFromUrdf(ch, *childRef, jointsOrder);
  }

  RAIFATAL_IF(jointsNumber != raiLink.childrenLinks_.size(),
              "URDF reader error. Please report to eastsky.kang@gmail.com")
}

void OdeArticulatedSystem::init() {
  // init recursively
  benchmark::Mat<3,3> baseRotMat;
  baseRotMat.setIdentity();
  benchmark::Vec<3> baseOrigin;
  baseOrigin.setZero();

  // init index of each body
  initIdx(rootLink_);

  // init ODE inertial properties
  initInertials(rootLink_);

  // init visual objects
  initVisuals(rootLink_, visObj, visProps_);

  // init ODE collision objects
  initCollisions(rootLink_, baseRotMat, baseOrigin, visColObj, visColProps_);

  // init ODE joints
  initJoints(rootLink_, baseRotMat, baseOrigin);

  // resize generalized states
  genForce_.resize(dof_);
  genForce_.setZero();
  genVelocity_.resize(dof_);
  genVelocity_.setZero();
  genCoordinate_.resize(stateDimension_);
  genCoordinate_.setZero();
}


void OdeArticulatedSystem::initIdx(Link &link) {
  link.bodyIdx_ = (int)links_.size();
  links_.push_back(&link);
  for(int i = 0; i < link.childrenLinks_.size(); i++) {
    link.childrenLinks_[i].parentIdx_ = link.bodyIdx_;
    initIdx(link.childrenLinks_[i]);
  }
}

void OdeArticulatedSystem::initVisuals(Link &link,
                                       std::vector<VisualObjectData> &collect,
                                       std::vector<VisualObjectProperty> &props) {
  // visual objects
  for(int i = 0; i < link.visual_.visshape_.size(); i++) {
    collect.emplace_back();
    collect.back() = std::make_tuple(link.visual_.visObjRotMat_[i],
                                     link.visual_.visObjOrigin_[i],
                                     link.bodyIdx_,
                                     link.visual_.visshape_[i],
                                     link.visual_.visColor_[i]);
    props.emplace_back(link.visual_.meshFileNames_[i],
                       link.visual_.visShapeParam_[i]);
  }

  // children
  for (auto &ch: link.childrenLinks_) {
    initVisuals(ch, collect, props);
  }
}

void OdeArticulatedSystem::initInertials(Link &link) {
  if(link.inertial_.mass_ == 0) {

    if(link.bodyIdx_ == 0 && isFixed_) {
      // fixed base link (set default values)
      dMassSetParameters(
          &link.inertial_.odeMass_,
          1,
          0,
          0,
          0,
          1,
          1,
          1,
          1,
          1,
          1
      );
      dBodySetMass(link.odeBody_, &link.inertial_.odeMass_);

    } else {
      // non-fixed-base link
      RAIFATAL("zero inertial non-base link is not allowed in ODE")
//      dBodySetKinematic(link.odeBody_);
    }
  }
  else {
    benchmark::Mat<3,3> inertia;
    benchmark::similarityTransform(link.inertial_.rotmat_, link.inertial_.inertia_, inertia);

    dMassSetParameters(
        &link.inertial_.odeMass_,
        link.inertial_.mass_,
        link.inertial_.pos_[0],
        link.inertial_.pos_[1],
        link.inertial_.pos_[2],
        inertia[0],
        inertia[4],
        inertia[8],
        inertia[1],
        inertia[2],
        inertia[7]
    );
    dBodySetMass(link.odeBody_, &link.inertial_.odeMass_);
  }

  for (auto &ch: link.childrenLinks_)
    initInertials(ch);
}

void OdeArticulatedSystem::initCollisions(Link &link,
                                          benchmark::Mat<3, 3> &parentRot_w,
                                          benchmark::Vec<3> &parentPos_w,
                                          std::vector<AlternativeVisualObjectData> &collect,
                                          std::vector<VisualObjectProperty> &props) {

  // collision objects
  for(int i = 0; i < link.collision_.colShape_.size(); i++) {
    benchmark::Vec<4> size = link.collision_.colShapeParam_[i];

    switch (link.collision_.colShape_[i]) {
      case bo::Shape::Cylinder: {
        link.collision_.odeGeometries_.push_back(dCreateCylinder(spaceID_, size[0], size[1]));
        dGeomSetBody(link.collision_.odeGeometries_.back(), link.odeBody_);
        dGeomSetCategoryBits(link.collision_.odeGeometries_.back(), collisionGroup_);
        dGeomSetCollideBits(link.collision_.odeGeometries_.back(), collisionMask_);
        break;
      }
      case bo::Shape::Sphere: {
        link.collision_.odeGeometries_.push_back(dCreateSphere(spaceID_, size[0]));
        dGeomSetBody(link.collision_.odeGeometries_.back(), link.odeBody_);
        dGeomSetCategoryBits(link.collision_.odeGeometries_.back(), collisionGroup_);
        dGeomSetCollideBits(link.collision_.odeGeometries_.back(), collisionMask_);
        break;
      }
      case bo::Shape::Box: {
        link.collision_.odeGeometries_.push_back(dCreateBox(spaceID_, size[0], size[1], size[2]));
        dGeomSetBody(link.collision_.odeGeometries_.back(), link.odeBody_);
        dGeomSetCategoryBits(link.collision_.odeGeometries_.back(), collisionGroup_);
        dGeomSetCollideBits(link.collision_.odeGeometries_.back(), collisionMask_);
        break;
      }
      default: {
        RAIFATAL("this shape of collision is not supported yet")
        break;
      }
    }

    // gyro mode
    dBodySetGyroscopicMode(link.odeBody_, true);

    // set body position
    dMatrix3 bodyR;
    for(int row = 0; row < 3; row++) {
      for(int col = 0; col < 3; col++) {
        bodyR[4*row + col] = parentRot_w[row + col*3];
      }
      bodyR[4*row + 3] = 0;
    }
    dBodySetPosition(link.odeBody_, parentPos_w[0], parentPos_w[1], parentPos_w[2]);
    dBodySetRotation(link.odeBody_, bodyR);

    // set each geometry position
    dMatrix3 geomR;
    for(int row = 0; row < 3; row++) {
      for(int col = 0; col < 3; col++) {
        geomR[4*row + col] = link.collision_.colObjRotMat_[i][row+col*3];
      }
      geomR[4*row + 3] = 0;
    }
    dGeomSetOffsetPosition(link.collision_.odeGeometries_.back(),
                           link.collision_.colObjOrigin_[i][0],
                           link.collision_.colObjOrigin_[i][1],
                           link.collision_.colObjOrigin_[i][2]);
    dGeomSetOffsetRotation(link.collision_.odeGeometries_.back(), geomR);

    // material properties
//    TODO
//    raiLink.matrialProps_.emplace_back();
//    dGeomSetData(raiLink.odeGeometries_.back(),
//                 raiLink.matrialProps_.back());

    // set alternative visualization object
    collect.emplace_back();
    collect.back() = std::make_tuple(link.collision_.colObjRotMat_[i],
                                     link.collision_.colObjOrigin_[i],
                                     link.bodyIdx_,
                                     link.collision_.colShape_[i]);
    props.emplace_back("",
                       link.collision_.colShapeParam_[i]);
  }
  // end of geometry

  // children
  for (auto &ch: link.childrenLinks_) {
    benchmark::Mat<3,3> rot_w;
    benchmark::Vec<3> pos_w;

    benchmark::matmul(parentRot_w, ch.parentJoint_.rotmat_, rot_w);
    benchmark::matvecmul(parentRot_w, ch.parentJoint_.pos_, pos_w);
    benchmark::vecadd(parentPos_w, pos_w);

    initCollisions(ch, rot_w, pos_w, collect, props);
  }
}

void OdeArticulatedSystem::initJoints(Link &link, benchmark::Mat<3, 3> &parentRot_w, benchmark::Vec<3> &parentPos_w) {

  if(link.bodyIdx_ == 0 && isFixed_) {
    // fixed base link
    rootJoint_.type = Joint::FIXED;
    rootJoint_.odeJoint_ = dJointCreateFixed(worldID_, 0);
    dJointAttach(rootJoint_.odeJoint_, link.odeBody_, 0);
    dJointSetFixed(rootJoint_.odeJoint_);
  }

  for(int i = 0; i < link.childrenLinks_.size(); i++) {
    Link &childLink = link.childrenLinks_[i];

    // joint position, axis, and orientation
    benchmark::Vec<3> pos_w;
    benchmark::Vec<3> axis_w;
    benchmark::Vec<3> temp;
    benchmark::Mat<3,3> rot_w;

    // orientation
    benchmark::matmul(parentRot_w, childLink.parentJoint_.rotmat_, rot_w);

    // axis
    benchmark::matvecmul(childLink.parentJoint_.rotmat_, childLink.parentJoint_.axis_, temp);
    benchmark::matvecmul(parentRot_w, temp, axis_w);

    // position
    benchmark::matvecmul(parentRot_w, childLink.parentJoint_.pos_, pos_w);
    benchmark::vecadd(parentPos_w, pos_w);

    switch (childLink.parentJoint_.type) {
      case Joint::FIXED: {
        childLink.parentJoint_.odeJoint_ = dJointCreateFixed(worldID_, 0);
        dJointAttach(childLink.parentJoint_.odeJoint_, link.odeBody_, childLink.odeBody_);
        dJointSetFixed(childLink.parentJoint_.odeJoint_);
        break;
      }
      case Joint::REVOLUTE: {
        // joint
        childLink.parentJoint_.odeJoint_ = dJointCreateHinge(worldID_, 0);
        dJointAttach(childLink.parentJoint_.odeJoint_, link.odeBody_, childLink.odeBody_);
        dJointSetHingeAnchor(
            childLink.parentJoint_.odeJoint_,
            pos_w[0],
            pos_w[1],
            pos_w[2]
        );
        dJointSetHingeAxis(
            childLink.parentJoint_.odeJoint_,
            axis_w[0],
            axis_w[1],
            axis_w[2]
        );

        dof_++;
        stateDimension_++;
        break;
      }
      case Joint::PRISMATIC: {
        childLink.parentJoint_.odeJoint_ = dJointCreateSlider(worldID_, 0);
        dJointAttach(childLink.parentJoint_.odeJoint_, link.odeBody_, childLink.odeBody_);
        dJointSetSliderAxis(
            childLink.parentJoint_.odeJoint_,
            axis_w[0],
            axis_w[1],
            axis_w[2]
        );

        dof_++;
        stateDimension_++;
        break;
      }
      default:
      RAIFATAL("currently only support revolute/prismatic/fixed joint");

    }

    // set data
    childLink.parentJoint_.childLink_ = &childLink;
    childLink.parentJoint_.gencoordIndex_ = stateDimension_ - 1;
    childLink.parentJoint_.genvelIndex_ = dof_ - 1;
    childLink.parentJoint_.jointId_ = (int)joints_.size();

    joints_.push_back(&childLink.parentJoint_);
    initJoints(childLink, rot_w, pos_w);
  }
}

OdeArticulatedSystem::~OdeArticulatedSystem() {
  for(int i = 0; i < links_.size(); i++) {

    // delete ode joints
    if(links_[i]->parentJoint_.odeJoint_)
      dJointDestroy(links_[i]->parentJoint_.odeJoint_);

    // delete ode body
    if(links_[i]->odeBody_)
      dBodyDestroy(links_[i]->odeBody_);

    // delete ode geom
    if(links_[i]->collision_.odeGeometries_.size() > 0) {
      for(int j = 0; j < links_[i]->collision_.odeGeometries_.size(); j++)
        dGeomDestroy(links_[i]->collision_.odeGeometries_[j]);
    }
  }
}

void OdeArticulatedSystem::updateJointPos(Link &link,
                                          benchmark::Mat<3, 3> &parentRot_w,
                                          benchmark::Vec<3> &parentPos_w) {

  // set ode body pos and rotation
  dMatrix3 bodyR;
  for(int row = 0; row < 3; row++) {
    for(int col = 0; col < 3; col++) {
      bodyR[4*row + col] = parentRot_w[row + col*3];
    }
    bodyR[4*row + 3] = 0;
  }

  // update base body position and rotation
  dBodySetPosition(link.odeBody_, parentPos_w[0], parentPos_w[1], parentPos_w[2]);
  dBodySetRotation(link.odeBody_, bodyR);

  // children
  for(int i = 0; i < link.childrenLinks_.size(); i++) {
    Link &childLink = link.childrenLinks_[i];

    // joint position, axis, and orientation
    benchmark::Vec<3> pos_w;
    benchmark::Vec<3> axis_w;
    benchmark::Vec<3> tempV;
    benchmark::Mat<3,3> rot_w;
    benchmark::Mat<3,3> jointR;
    benchmark::Mat<3,3> tempR;

    switch (childLink.parentJoint_.type) {
      case Joint::FIXED: {
        benchmark::matmul(parentRot_w, childLink.parentJoint_.rotmat_, rot_w);
        benchmark::matvecmul(parentRot_w, childLink.parentJoint_.pos_, pos_w);
        benchmark::vecadd(parentPos_w, pos_w);
        break;
      }
      case Joint::REVOLUTE: {
        // orientation
        benchmark::angleAxisToRotMat(
            childLink.parentJoint_.axis_,
            genCoordinate_[childLink.parentJoint_.gencoordIndex_],
            jointR
        );
        benchmark::matmul(childLink.parentJoint_.rotmat_, jointR, tempR);
        benchmark::matmul(parentRot_w, tempR, rot_w);

        // axis
        benchmark::matvecmul(childLink.parentJoint_.rotmat_, childLink.parentJoint_.axis_, tempV);
        benchmark::matvecmul(parentRot_w, tempV, axis_w);

        // position
        benchmark::matvecmul(parentRot_w, childLink.parentJoint_.pos_, pos_w);
        benchmark::vecadd(parentPos_w, pos_w);
        break;
      }
      case Joint::PRISMATIC: {
        // orientation
        benchmark::matmul(parentRot_w, childLink.parentJoint_.rotmat_, rot_w);

        // axis
        benchmark::Vec<3> tempV2;
        benchmark::matvecmul(childLink.parentJoint_.rotmat_, childLink.parentJoint_.axis_, tempV2);

        // position
        benchmark::vecScalarMul(
            genCoordinate_[childLink.parentJoint_.gencoordIndex_],
            tempV2,
            tempV
        );

        benchmark::vecadd(childLink.parentJoint_.pos_, tempV);

        benchmark::matvecmul(parentRot_w, tempV, pos_w);
        benchmark::vecadd(parentPos_w, pos_w);
        break;
      }
      default:
      RAIFATAL("currently only support revolute/prismatic/fixed joint");
    }

    updateJointPos(childLink, rot_w, pos_w);
  }
}

const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedCoordinate() {
  if(isFixed_) {
    // fixed body
    int i = 0;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          genCoordinate_[i++] = -dJointGetHingeAngle(joint->odeJoint_);;
          break;
        }
        case Joint::PRISMATIC: {
          RAIFATAL("prismatic joint is not tested yet")
          genCoordinate_[i++] = dJointGetSliderPosition(joint->odeJoint_);
          break;
        }
        default:
        RAIINFO("not supported joint type")
      }
    }
  }
  else {
    // floating body
    const dReal *position = dBodyGetPosition(rootLink_.odeBody_);
    const dReal *quaternion = dBodyGetQuaternion(rootLink_.odeBody_);

    genCoordinate_[0] = position[0];
    genCoordinate_[1] = position[1];
    genCoordinate_[2] = position[2];
    genCoordinate_[3] = quaternion[0];
    genCoordinate_[4] = quaternion[1];
    genCoordinate_[5] = quaternion[2];
    genCoordinate_[6] = quaternion[3];

    int i = 7;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          genCoordinate_[i++] = -dJointGetHingeAngle(joint->odeJoint_);
          break;
        }
        case Joint::PRISMATIC: {
          RAIFATAL("prismatic joint is not tested yet")
          genCoordinate_[i++] = dJointGetSliderPosition(joint->odeJoint_);
          break;
        }
        default:
        RAIINFO("not supported joint type")
      }
    }
  }
  return genCoordinate_.e();
}

const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedVelocity() {
  if(isFixed_) {
    // fixed body
    int i = 0;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          genVelocity_[i++] = -dJointGetHingeAngleRate(joint->odeJoint_);;
          break;
        }
        case Joint::PRISMATIC: {
          RAIFATAL("prismatic joint is not tested yet")
          genVelocity_[i++] = dJointGetSliderPositionRate(joint->odeJoint_);
          break;
        }
        default:
        RAIINFO("not supported joint type")
      }
    }
  }
  else {
    // floating body
    const dReal *linvel = dBodyGetLinearVel(rootLink_.odeBody_);
    const dReal *angvel = dBodyGetAngularVel(rootLink_.odeBody_);

    genVelocity_[0] = linvel[0];
    genVelocity_[1] = linvel[1];
    genVelocity_[2] = linvel[2];
    genVelocity_[3] = angvel[0];
    genVelocity_[4] = angvel[1];
    genVelocity_[5] = angvel[2];

    int i = 6;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          genVelocity_[i++] = -dJointGetHingeAngleRate(joint->odeJoint_);;
          break;
        }
        case Joint::PRISMATIC: {
          RAIFATAL("prismatic joint is not tested yet")
          genVelocity_[i++] = dJointGetSliderPositionRate(joint->odeJoint_);
          break;
        }
        default:
        RAIINFO("not supported joint type")
      }
    }
  }
  return genVelocity_.e();
}

void OdeArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {
  RAIFATAL("not implemented yet")
}

void OdeArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
  RAIFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input")

  for(int i = 0; i < stateDimension_; i++) {
    genCoordinate_[i] = jointState[i];
  }

  if(isFixed_) {
    // fixed body
    benchmark::Mat<3,3> baseRotMat;
    baseRotMat.setIdentity();
    benchmark::Vec<3> baseOrigin;
    baseOrigin.setZero();

    updateJointPos(rootLink_, baseRotMat, baseOrigin);
  }
  else {
    // floating body
    benchmark::Mat<3,3> baseRotMat;
    benchmark::Vec<4> baseQuat;
    baseQuat = {
      jointState[3],
      jointState[4],
      jointState[5],
      jointState[6],
    };
    benchmark::quatToRotMat(baseQuat, baseRotMat);

    benchmark::Vec<3> baseOrigin;
    baseOrigin = {
        jointState[0],
        jointState[1],
        jointState[2]
    };

    updateJointPos(rootLink_, baseRotMat, baseOrigin);
  }
}

void OdeArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {
  RAIFATAL_IF(jointState.size() != stateDimension_, "invalid generalized coordinate input")

  for(int i = 0; i < stateDimension_; i++) {
    genCoordinate_[i] = jointState.begin()[i];
  }

  if(isFixed_) {
    // fixed body
    benchmark::Mat<3,3> baseRotMat;
    baseRotMat.setIdentity();
    benchmark::Vec<3> baseOrigin;
    baseOrigin.setZero();

    updateJointPos(rootLink_, baseRotMat, baseOrigin);
  }
  else {
    // floating body
    benchmark::Mat<3,3> baseRotMat;
    benchmark::Vec<4> baseQuat;
    baseQuat = {
        jointState.begin()[3],
        jointState.begin()[4],
        jointState.begin()[5],
        jointState.begin()[6],
    };
    benchmark::quatToRotMat(baseQuat, baseRotMat);

    benchmark::Vec<3> baseOrigin;
    baseOrigin = {
        jointState.begin()[0],
        jointState.begin()[1],
        jointState.begin()[2]
    };

    updateJointPos(rootLink_, baseRotMat, baseOrigin);
  }
}

void OdeArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  if(isFixed_) {
    // fixed body
    int i = 0;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          dJointAddHingeTorque(joint->odeJoint_, -tau.begin()[i++]);
          break;
        }
        case Joint::PRISMATIC: {
          RAIFATAL("prismatic joint is not tested yet")
          dJointAddSliderForce(joint->odeJoint_, tau.begin()[i++]);
          break;
        }
        default:
        RAIINFO("not supported joint type")
      }
    }
  }
  else {
    // floating body
    dBodyAddForce(rootLink_.odeBody_, tau.begin()[0], tau.begin()[1], tau.begin()[2]);
    dBodyAddTorque(rootLink_.odeBody_, tau.begin()[3], tau.begin()[4], tau.begin()[4]);

    int i = 6;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          dJointAddHingeTorque(joint->odeJoint_, -tau.begin()[i++]);
          break;
        }
        case Joint::PRISMATIC: {
          RAIFATAL("prismatic joint is not tested yet")
          dJointAddSliderForce(joint->odeJoint_, tau.begin()[i++]);
          break;
        }
        default:
        RAIINFO("not supported joint type")
      }
    }
  }
}

void OdeArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {
  RAIFATAL_IF(tau.size() != dof_, "invalid generalized force input")
  if(isFixed_) {
    // fixed body
    int i = 0;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          dJointAddHingeTorque(joint->odeJoint_, -tau[i++]);
          break;
        }
        case Joint::PRISMATIC: {
          RAIFATAL("prismatic joint is not tested yet")
          dJointAddSliderForce(joint->odeJoint_, tau[i++]);
          break;
        }
        default:
        RAIINFO("not supported joint type")
      }
    }
  }
  else {
    // floating body
    dBodyAddForce(rootLink_.odeBody_, tau[0], tau[1], tau[2]);
    dBodyAddTorque(rootLink_.odeBody_, tau[3], tau[4], tau[4]);

    int i = 6;
    for(auto *joint: joints_) {
      switch (joint->type) {
        case Joint::FIXED: {
          continue;
        }
        case Joint::REVOLUTE: {
          dJointAddHingeTorque(joint->odeJoint_, -tau[i++]);
          break;
        }
        case Joint::PRISMATIC: {
          RAIFATAL("prismatic joint is not tested yet")
          dJointAddSliderForce(joint->odeJoint_, tau[i++]);
          break;
        }
        default:
        RAIINFO("not supported joint type")
      }
    }
  }
}

void OdeArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
  setGeneralizedCoordinate(genco);
  setGeneralizedVelocity(genvel);
}

const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedForce() {
  RAIFATAL("not implemented yet")
  return genForce_.e();
}

int OdeArticulatedSystem::getDOF() {
  return dof_;
}

int OdeArticulatedSystem::getStateDimension() {
  return stateDimension_;
}

void OdeArticulatedSystem::setColor(Eigen::Vector4d color) {
  color_ = {
      color[0], color[1], color[2], color[3]};

  for(int i = 0; i < visObj.size(); i++) {
    std::get<4>(visObj[i]) = color_;
  }
}

const std::vector<Joint *> &OdeArticulatedSystem::getJoints() const {
  return joints_;
}

const std::vector<Link *> &OdeArticulatedSystem::getLinks() const {
  return links_;
}

void OdeArticulatedSystem::getBodyPose(int bodyId, benchmark::Mat<3, 3> &orientation, benchmark::Vec<3> &position) {
  Link *link = links_[bodyId];

  const dReal *pos = dBodyGetPosition(link->odeBody_);
  position = {pos[0], pos[1], pos[2]};

  const dReal* rot = dBodyGetRotation(link->odeBody_);
  orientation.e() << rot[0], rot[1], rot[2],
      rot[4], rot[5], rot[6],
      rot[8], rot[9], rot[10];
}

void OdeArticulatedSystem::getComVelocity_W(int bodyId, benchmark::Vec<3> &velocity) {
  dReal comvel[3];
  benchmark::Mat<3, 3> bodyOrientation;
  benchmark::Vec<3> comPosition;
  getBodyPose(bodyId, bodyOrientation, comPosition);

  benchmark::matvecmulThenAdd(bodyOrientation, links_[bodyId]->inertial_.pos_, comPosition);
  dBodyGetPointVel(links_[bodyId]->odeBody_,
                   comPosition[0],
                   comPosition[1],
                   comPosition[2],
                   comvel);

  velocity = {comvel[0], comvel[1], comvel[2]};
}

const Eigen::Map<Eigen::Matrix<double, 3, 1>> OdeArticulatedSystem::getLinearMomentumInCartesianSpace() {
  linearMomentum_.setZero();
  for(int i = 0; i < links_.size(); i++) {
    benchmark::Vec<3> comvel;
    getComVelocity_W(i, comvel);
    benchmark::vecScalarMul(links_[i]->inertial_.odeMass_.mass, comvel);
    benchmark::vecadd(comvel, linearMomentum_);
  }
  return linearMomentum_.e();
}
double OdeArticulatedSystem::getTotalMass() {
  double totalMass = 0;
  for(int i = 0; i < links_.size(); i++) {
    totalMass += links_[i]->inertial_.odeMass_.mass;
  }
  return totalMass;
}

void OdeArticulatedSystem::getBodyOmega_W(int bodyId, benchmark::Vec<3> &omega) {
  const dReal *angvel = dBodyGetAngularVel(links_[bodyId]->odeBody_);
  benchmark::Mat<3, 3> bodyOrientation;
  benchmark::Vec<3> temp;
  getBodyPose(bodyId, bodyOrientation, temp); // temp here is just for dummy

  temp = {angvel[0], angvel[1], angvel[2]};
  benchmark::matvecmul(bodyOrientation, temp, omega);
}

void OdeArticulatedSystem::getComPos_W(int bodyId, benchmark::Vec<3> &comPos) {
  benchmark::Vec<3> bodyPos;
  benchmark::Mat<3, 3> bodyOrientation;
  getBodyPose(bodyId, bodyOrientation, bodyPos);

  comPos = bodyPos;
  benchmark::matvecmulThenAdd(bodyOrientation, links_[bodyId]->inertial_.pos_, comPos);
}

double OdeArticulatedSystem::getEnergy(const benchmark::Vec<3> &gravity) {
  double kinetic = 0;
  for(int i = 0; i < links_.size(); i++) {
    benchmark::Vec<3> comvel;
    getComVelocity_W(i, comvel);
    kinetic += pow(comvel.norm(), 2) * links_[i]->inertial_.odeMass_.mass;

    double angular;
    benchmark::Vec<3> omega;
    getBodyOmega_W(i, omega);
    benchmark::vecTransposeMatVecMul(omega, links_[i]->inertial_.inertia_, angular);
    kinetic += angular;
  }

  double potential = 0;
  for(int i = 0; i < links_.size(); i++) {
    double linkPotential = 0;
    benchmark::Vec<3> temp;
    getComPos_W(i, temp);
    benchmark::vecDot(gravity, temp, linkPotential);
    potential -= linkPotential * links_[i]->inertial_.mass_;
  }

  return potential + 0.5 * kinetic;
}
void OdeArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {
  RAIFATAL("not implemented yet")
//  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
//  if(isFixed_) {
//    // fixed body
//    for(auto *joint: joints_) {
//      double rate = jointVel[joint->genvelIndex_];
//
//      switch (joint->type) {
//        case Joint::FIXED: {
//          continue;
//        }
//        case Joint::REVOLUTE: {
//          dReal axis[3];
//          dJointGetHingeAxis(joint->odeJoint_, axis);
//          dBodySetAngularVel(joint->childLink_->odeBody_,
//                             axis[0] * rate,
//                             axis[1] * rate,
//                             axis[2] * rate);
//          break;
//        }
//        case Joint::PRISMATIC: {
//          RAIFATAL("prismatic joint is not tested yet")
////          dJointAddSliderForce(joint->odeJoint_, tau.begin()[i++]);
//          break;
//        }
//        default:
//        RAIINFO("not supported joint type")
//      }
//    }
//  }
//  else {
//    // floating body
//    dBodySetLinearVel(rootLink_.odeBody_,
//                      jointVel[0],
//                      jointVel[1],
//                      jointVel[2]);
//    dBodySetAngularVel(rootLink_.odeBody_,
//                       jointVel[3],
//                       jointVel[4],
//                       jointVel[5]);
//
//    for(auto *joint: joints_) {
//      double rate = jointVel[joint->genvelIndex_];
//
//      switch (joint->type) {
//        case Joint::FIXED: {
//          continue;
//        }
//        case Joint::REVOLUTE: {
//          dReal axis[3];
//          dJointGetHingeAxis(joint->odeJoint_, axis);
//          dBodySetAngularVel(joint->childLink_->odeBody_,
//                             axis[0] * rate,
//                             axis[1] * rate,
//                             axis[2] * rate);
//          break;
//        }
//        case Joint::PRISMATIC: {
//          RAIFATAL("prismatic joint is not tested yet")
////          dJointAddSliderForce(joint->odeJoint_, tau.begin()[i++]);
//          break;
//        }
//        default:
//        RAIINFO("not supported joint type")
//      }
//    }
//  }
}
void OdeArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {
  RAIFATAL("not implemented yet")
//  RAIFATAL_IF(jointVel.size() != dof_, "invalid generalized velocity input")
//  if(isFixed_) {
//    // fixed body
//    for(auto *joint: joints_) {
//      double rate = jointVel.begin()[joint->genvelIndex_];
//
//      switch (joint->type) {
//        case Joint::FIXED: {
//          continue;
//        }
//        case Joint::REVOLUTE: {
//          dReal axis[3];
//          dJointGetHingeAxis(joint->odeJoint_, axis);
//          dBodySetAngularVel(joint->childLink_->odeBody_,
//                             axis[0] * rate,
//                             axis[1] * rate,
//                             axis[2] * rate);
//          break;
//        }
//        case Joint::PRISMATIC: {
//          RAIFATAL("prismatic joint is not tested yet")
////          dJointAddSliderForce(joint->odeJoint_, tau.begin()[i++]);
//          break;
//        }
//        default:
//        RAIINFO("not supported joint type")
//      }
//    }
//  }
//  else {
//    // floating body
//    dBodySetLinearVel(rootLink_.odeBody_,
//                      jointVel.begin()[0],
//                      jointVel.begin()[1],
//                      jointVel.begin()[2]);
//    dBodySetAngularVel(rootLink_.odeBody_,
//                       jointVel.begin()[3],
//                       jointVel.begin()[4],
//                       jointVel.begin()[5]);
//
//    for(auto *joint: joints_) {
//      double rate = jointVel.begin()[joint->genvelIndex_];
//
//      switch (joint->type) {
//        case Joint::FIXED: {
//          continue;
//        }
//        case Joint::REVOLUTE: {
//          dReal axis[3];
//          dJointGetHingeAxis(joint->odeJoint_, axis);
//          dBodySetAngularVel(joint->childLink_->odeBody_,
//                             axis[0] * rate,
//                             axis[1] * rate,
//                             axis[2] * rate);
//
//          const dReal *vel = dBodyGetAngularVel(joint->childLink_->odeBody_);
//          RAIINFO(vel[0] << " " << vel[1] << " " << vel[2])
//          break;
//        }
//        case Joint::PRISMATIC: {
//          RAIFATAL("not implemented yet")
//          break;
//        }
//        default:
//        RAIINFO("not supported joint type")
//      }
//    }
//  }
}

} // object
} // ode_sim