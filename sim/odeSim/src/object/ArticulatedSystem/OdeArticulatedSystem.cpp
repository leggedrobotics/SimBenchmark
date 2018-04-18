//
// Created by kangd on 15.04.18.
//

#include "OdeArticulatedSystem.hpp"

namespace ode_sim {
namespace object {

//void Link::initCollisions(std::vector<AlternativeVisualObjectData> &collect,
//                          std::vector<std::pair<std::string, benchmark::Vec<4>>> &props) {
//  // visual objects
//  for(int i = 0; i < visshape_.size(); i++) {
//    collect.emplace_back();
//    collect.back() = std::make_tuple(colObjRotMat_[i], colObjOrigin_[i], bodyIdx_, colShape_[i]);
//    props.emplace_back("", colShapeParam_[i]);
//  }
//
//  // children
//  for (auto &ch: childrenLinks_)
//    ch.initCollisions(collect, props);
//}

OdeArticulatedSystem::OdeArticulatedSystem(std::string urdfFile,
                                           const dWorldID worldID,
                                           const dSpaceID spaceID)
    : worldID_(worldID), spaceID_(spaceID) {

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
    isFixed_ = false;
//    baseType_ = Joint::FLOATING;
//    dof = 5;
//    jointStateDim = 6;
//    baseJointStateDimMinusOne = 6;
//    baseDOFminusOne = 5;
  } else {
    isFixed_ = false;
//    baseType_ = Joint::FIXED;
//    jointStateDim = -1;
//    baseJointStateDimMinusOne = -1;
//    baseDOFminusOne = -1;
    if (rootLink->child_links[0]->parent_joint->type == urdf::Joint::FIXED)
      rootLink = rootLink->child_links[0];
  }

  links_.emplace_back();

  // process links recursively
  processLinkFromUrdf(rootLink, links_.back(), jointsNames_);

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

    switch (jnt->type) {
      case urdf::Joint::FIXED: {
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::FIXED;
//        childRef->joint.odeJoint_ = dJointCreateFixed(worldID_, 0);
        break;
      }
      case urdf::Joint::REVOLUTE: {
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::REVOLUTE;
//        childRef->joint.odeJoint_ = dJointCreateHinge(worldID_, 0);
        break;
      }
      case urdf::Joint::PRISMATIC: {
        raiLink.childrenLinks_.emplace_back();
        childRef = &raiLink.childrenLinks_.back();
        childRef->parentJoint_.type = Joint::PRISMATIC;
//        childRef->joint.odeJoint_ = dJointCreateSlider(worldID_, 0);
        break;
      }
      default:
      RAIFATAL("currently only support revolute/prismatic/fixed joint");
    }

    double r, p, y;
    jnt->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
    benchmark::Vec<3> rpy;
    rpy.e() << r, p, y;
    childRef->RPY(rpy);
    double axisNorm = sqrt(jnt->axis.x * jnt->axis.x + jnt->axis.y * jnt->axis.y + jnt->axis.z * jnt->axis.z);
    childRef->parentJoint_.jointAxis({jnt->axis.x / axisNorm, jnt->axis.y / axisNorm, jnt->axis.z / axisNorm});
    childRef->parentJoint_.jointPosition({jnt->parent_to_joint_origin_transform.position.x,
                                          jnt->parent_to_joint_origin_transform.position.y,
                                          jnt->parent_to_joint_origin_transform.position.z});

    processLinkFromUrdf(ch, *childRef, jointsOrder);
  }

  RAIFATAL_IF(jointsNumber != raiLink.childrenLinks_.size(),
              "URDF reader error. Please report to eastsky.kang@gmail.com")
}

void OdeArticulatedSystem::init() {
  genForce_.resize(dof_);
  genForce_.setZero();
  genVelocity_.resize(dof_);
  genVelocity_.setZero();
  genCoordinate_.resize(stateDimension_);
  genCoordinate_.setZero();

  // init recursively
  benchmark::Mat<3,3> baseRotMat;
  baseRotMat.setIdentity();
  benchmark::Vec<3> baseOrigin;
  baseOrigin.setZero();

  initInertial(links_[0]);
  initVisuals(links_[0], baseRotMat, baseOrigin, visObj, visProps_);
//  links_[0].initCollisions(visColObj, visColProps_);
}

void OdeArticulatedSystem::initVisuals(Link &link,
                                       benchmark::Mat<3, 3> &parentRot_w,
                                       benchmark::Vec<3> &parentPos_w,
                                       std::vector<VisualObjectData> &collect,
                                       std::vector<VisualObjectProperty> &props) {
  // visual objects
  for(int i = 0; i < link.visual_.visshape_.size(); i++) {
    benchmark::Mat<3,3> rot_w;
    benchmark::Vec<3> pos_w;

    benchmark::matmul(parentRot_w, link.visual_.visObjRotMat_[i], rot_w);
    benchmark::matvecmul(parentRot_w, link.visual_.visObjOrigin_[i], pos_w);
    benchmark::vecadd(parentPos_w, pos_w);

    collect.emplace_back();
    collect.back() = std::make_tuple(rot_w,
                                     pos_w,
                                     link.bodyIdx_,
                                     link.visual_.visshape_[i],
                                     link.visual_.visColor_[i]);
    props.emplace_back(link.visual_.meshFileNames_[i],
                       link.visual_.visShapeParam_[i]);
  }

  // children
  for (auto &ch: link.childrenLinks_) {
    benchmark::Mat<3,3> rot_w;
    benchmark::Vec<3> pos_w;

    benchmark::matmul(parentRot_w, ch.rotMat_B_, rot_w);
    benchmark::matvecmul(parentRot_w, ch.parentJoint_.pos_, pos_w);
    benchmark::vecadd(parentPos_w, pos_w);

    initVisuals(ch, rot_w, pos_w, collect, props);
  }
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
        break;
      }
      case bo::Shape::Sphere: {
        link.collision_.odeGeometries_.push_back(dCreateSphere(spaceID_, size[0]));
        dGeomSetBody(link.collision_.odeGeometries_.back(), link.odeBody_);
        break;
      }
      case bo::Shape::Box: {
        link.collision_.odeGeometries_.push_back(dCreateBox(spaceID_, size[0], size[1], size[2]));
        dGeomSetBody(link.collision_.odeGeometries_.back(), link.odeBody_);
        break;
      }
      default: {
        RAIFATAL("this shape of collision is not supported yet")
        break;
      }
    }

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
    benchmark::Mat<3,3> rot_w;
    benchmark::Vec<3> pos_w;

    benchmark::matmul(parentRot_w, link.collision_.colObjRotMat_[i], rot_w);
    benchmark::matvecmul(parentRot_w, link.collision_.colObjOrigin_[i], pos_w);
    benchmark::vecadd(parentPos_w, pos_w);

    collect.emplace_back();
    collect.back() = std::make_tuple(rot_w,
                                     pos_w,
                                     link.bodyIdx_,
                                     link.collision_.colShape_[i]);
    props.emplace_back(link.visual_.meshFileNames_[i],
                       link.visual_.visShapeParam_[i]);

  }
  // end of geometry

  // children
  for (auto &ch: link.childrenLinks_) {
    benchmark::Mat<3,3> rot_w;
    benchmark::Vec<3> pos_w;

    benchmark::matmul(parentRot_w, ch.rotMat_B_, rot_w);
    benchmark::matvecmul(parentRot_w, ch.parentJoint_.pos_, pos_w);
    benchmark::vecadd(parentPos_w, pos_w);

    initCollisions(ch, rot_w, pos_w, collect, props);
  }
}

void OdeArticulatedSystem::initInertial(Link &link) {
  if(link.inertial_.mass_ == 0) {
    link.inertial_.odeMass_.setZero();
  } else {
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
    initInertial(ch);
}
OdeArticulatedSystem::~OdeArticulatedSystem() {
  for(int i = 0; i < links_.size(); i++) {
    // TODO
    // delete ode body
    // delete ode geom
    // delete ode joints
  }
}
const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedCoordinate() {
  return genCoordinate_.e();
}
const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedVelocity() {
  return genVelocity_.e();
}
void OdeArticulatedSystem::setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {

}
void OdeArticulatedSystem::setGeneralizedVelocity(const Eigen::VectorXd &jointVel) {

}
void OdeArticulatedSystem::setGeneralizedCoordinate(std::initializer_list<double> jointState) {

}
void OdeArticulatedSystem::setGeneralizedVelocity(std::initializer_list<double> jointVel) {

}
void OdeArticulatedSystem::setGeneralizedForce(std::initializer_list<double> tau) {

}

void OdeArticulatedSystem::getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) {

}

void OdeArticulatedSystem::setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {

}
void OdeArticulatedSystem::setGeneralizedForce(const Eigen::VectorXd &tau) {

}
const benchmark::object::ArticulatedSystemInterface::EigenVec OdeArticulatedSystem::getGeneralizedForce() {
  return genForce_.e();
}
int OdeArticulatedSystem::getDOF() {
  return dof_;
}
void OdeArticulatedSystem::setColor(Eigen::Vector4d color) {

}
} // object
} // ode_sim