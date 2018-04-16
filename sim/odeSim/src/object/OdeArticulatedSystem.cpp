//
// Created by kangd on 15.04.18.
//

#include "OdeArticulatedSystem.hpp"

namespace ode_sim {
namespace object {

OdeArticulatedSystem::OdeArticulatedSystem(std::string urdfFile,
                                           const dWorldID worldID,
                                           const dSpaceID spaceID)
    : worldID_(worldID), spaceID_(spaceID) {

//  urdfFile += "robot.urdf";
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

//  frameOfInterest_.resize(jointsNames_.size());
//
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
  processLinkFromUrdf(rootLink, links_.back(), jointsNames_);
  init();
}

void OdeArticulatedSystem::processLinkFromUrdf(boost::shared_ptr<const urdf::Link> urdfLink,
                                               Link &raiLink,
                                               std::vector<std::string> &jointsOrder) {

  // link init
  raiLink.odeBody_ = dBodyCreate(worldID_);
  raiLink.name = urdfLink->name;

  /// parent
  if(urdfLink->getParent()) {
    raiLink.parentJointName = urdfLink->parent_joint->name;
    raiLink.parentName = urdfLink->getParent()->name;
  } else {
    raiLink.parentJointName = "";
    raiLink.parentName = "";
  }

  /// inertial
  if (urdfLink->inertial) {
    dMassSetParameters(
        &raiLink.odeMass_,
        urdfLink->inertial->mass,
        urdfLink->inertial->origin.position.x,
        urdfLink->inertial->origin.position.y,
        urdfLink->inertial->origin.position.z,
        urdfLink->inertial->ixx, urdfLink->inertial->iyy, urdfLink->inertial->izz,
        urdfLink->inertial->ixy, urdfLink->inertial->ixz, urdfLink->inertial->iyz
    );
    dBodySetMass(raiLink.odeBody_, &raiLink.odeMass_);
  }
  else {
    raiLink.odeMass_.setZero();
  }

  /// collision
  if (urdfLink->collision) {
    // collision object
    int idx = 0;
    for (auto &col: urdfLink->collision_array) {
      if (col->geometry->type == urdf::Geometry::BOX) {
        // box
        auto box = boost::dynamic_pointer_cast<urdf::Box>(col->geometry);
        raiLink.odeGeometries_.push_back(dCreateBox(spaceID_, box->dim.x, box->dim.y, box->dim.z));
        dGeomSetBody(raiLink.odeGeometries_.back(), raiLink.odeBody_);
      }
      else if (col->geometry->type == urdf::Geometry::CYLINDER) {
        // cylinder
        auto cyl = boost::dynamic_pointer_cast<urdf::Cylinder>(col->geometry);
        raiLink.odeGeometries_.push_back(dCreateCylinder(spaceID_, cyl->radius, cyl->length));
        dGeomSetBody(raiLink.odeGeometries_.back(), raiLink.odeBody_);
      }
      else if (col->geometry->type == urdf::Geometry::SPHERE) {
        // sphere
        auto sph = boost::dynamic_pointer_cast<urdf::Sphere>(col->geometry);
        raiLink.odeGeometries_.push_back(dCreateSphere(spaceID_, sph->radius));
        dGeomSetBody(raiLink.odeGeometries_.back(), raiLink.odeBody_);
      }
      else {
        RAIFATAL("mesh collision body is not supported yet");
      }

      // relative position
      auto &pos = col->origin.position;
      double r, p, y;
      benchmark::Mat<3,3> rotmat;
      col->origin.rotation.getRPY(r, p, y);
      benchmark::rpyToRotMat_intrinsic({r, p, y}, rotmat);

      dMatrix3 drotation;
      for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
          drotation[4*i + j] = rotmat[i+j*3];
        }
        drotation[4*i + 3] = 0;
      }

      dGeomSetOffsetPosition(raiLink.odeGeometries_.back(), pos.x, pos.y, pos.z);
      dGeomSetOffsetRotation(raiLink.odeGeometries_.back(), drotation);
    }
  }

  /// visual
  if (urdfLink->visual) {
    for (auto &vis: urdfLink->visual_array) {
      if (vis->geometry->type == urdf::Geometry::BOX) {
        // box
        auto box = boost::dynamic_pointer_cast<urdf::Box>(vis->geometry);
        raiLink.visshape.push_back(bo::Shape::Box);
        raiLink.visShapeParam.push_back({box->dim.x, box->dim.y, box->dim.z, 0});
        raiLink.meshFileNames.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::CYLINDER) {
        // cylinder
        auto cyl = boost::dynamic_pointer_cast<urdf::Cylinder>(vis->geometry);
        raiLink.visshape.push_back(bo::Shape::Cylinder);
        raiLink.visShapeParam.push_back({cyl->radius, cyl->length, 0, 0});
        raiLink.meshFileNames.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::SPHERE) {
        // sphere
        auto sph = boost::dynamic_pointer_cast<urdf::Sphere>(vis->geometry);
        raiLink.visshape.push_back(bo::Shape::Sphere);
        raiLink.visShapeParam.push_back({sph->radius, 0, 0, 0});
        raiLink.meshFileNames.emplace_back("");
      }
      else if (vis->geometry->type == urdf::Geometry::MESH) {
        // mesh
        auto mes = boost::dynamic_pointer_cast<urdf::Mesh>(vis->geometry);
        raiLink.visshape.push_back(bo::Shape::Mesh);
        raiLink.visShapeParam.push_back({mes->scale.x, mes->scale.y, mes->scale.z, 0});
        raiLink.meshFileNames.push_back(mes->filename);
      }

      if (vis->material)
        raiLink.visColor.push_back({vis->material->color.r, vis->material->color.g, vis->material->color.b, vis->material->color.a});
      else
        raiLink.visColor.push_back({0.7, 0.7, 0.7, 1.0});

      auto &pos = vis->origin.position;
      double r, p, y;
      vis->origin.rotation.getRPY(r, p, y);
      raiLink.visObjOrigin.push_back({pos.x, pos.y, pos.z});
      raiLink.visOrientation({r, p, y});
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
        raiLink.fixedBodies.emplace_back();
        childRef = &raiLink.fixedBodies.back();
        break;
      }
      case urdf::Joint::REVOLUTE: {
        raiLink.childrenLinks.emplace_back();
        childRef = &raiLink.childrenLinks.back();
        childRef->joint.type = Joint::REVOLUTE;
        break;
      }
      case urdf::Joint::PRISMATIC: {
        raiLink.childrenLinks.emplace_back();
        childRef = &raiLink.childrenLinks.back();
        childRef->joint.type = Joint::PRISMATIC;
        break;
      }
      default:
      RAIFATAL("currently only support revolute/prismatic/fixed joint");
    }

    processLinkFromUrdf(ch, *childRef, jointsOrder);
  }

  RAIFATAL_IF(jointsNumber != raiLink.childrenLinks.size() + raiLink.fixedBodies.size(),
              "URDF reader error. Please report to eastsky.kang@gmail.com")
}

void OdeArticulatedSystem::init() {
  genForce_.resize(dof_);
  genForce_.setZero();
  genVelocity_.resize(dof_);
  genVelocity_.setZero();
  genCoordinate_.resize(stateDimension_);
  genCoordinate_.setZero();

  links_[0].initVisuals(visObj, visProps_);
}

void Link::initVisuals(std::vector<std::tuple<benchmark::Mat<3, 3>,
                                              benchmark::Vec<3>,
                                              int,
                                              bo::Shape,
                                              benchmark::Vec<4>>> &collect,
                       std::vector<std::pair<std::string, benchmark::Vec<4>>> &props) {

  for(int i = 0; i < visshape.size(); i++) {
    collect.emplace_back();
    collect.back() = std::make_tuple(visObjRotMat[i], visObjOrigin[i], bodyIdx, visshape[i], visColor[i]);
    props.emplace_back(meshFileNames[i], visShapeParam[i]);
  }

  for (auto &ch: childrenLinks)
    ch.initVisuals(collect, props);
}

OdeArticulatedSystem::~OdeArticulatedSystem() {
  for(int i = 0; i < links_.size(); i++) {
    // TODO
    // delete ode
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