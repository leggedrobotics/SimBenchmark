//
// Created by kangd on 13.02.18.
//

#include "ArticulatedSystem.hpp"

namespace bullet_sim {
namespace object {


ArticulatedSystem::ArticulatedSystem(std::string urdfFile, std::vector<std::string> jointOrder) {

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
//
//  unsigned long searchPosition = 0;
//  jointsNames_ = jointOrder;
//
//  while (true) {
//    unsigned long position = model_xml_string.find("joint name", searchPosition);
//    if (position == std::string::npos) break;
//    unsigned long nameStart = model_xml_string.find("\"", position);
//    unsigned long nameEnd = model_xml_string.find("\"", nameStart + 1);
//
//    if (std::find(jointsNames_.begin(), jointsNames_.end(), model_xml_string.substr(nameStart + 1, nameEnd - nameStart - 1)) == jointsNames_.end())
//      jointsNames_.push_back(model_xml_string.substr(nameStart + 1, nameEnd - nameStart - 1));
//    searchPosition = nameEnd;
//  }
//
//  frameOfInterest_.resize(jointsNames_.size());
//
//  boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDFFile(urdfFile);
//  boost::shared_ptr<const urdf::Link> rootLink = urdf_model->getRoot();
//
//  if (rootLink->name != "world") {
//    baseType_ = Joint::FLOATING;
//    dof = 5;
//    jointStateDim = 6;
//    baseJointStateDimMinusOne = 6;
//    baseDOFminusOne = 5;
//  } else {
//    baseType_ = Joint::FIXED;
//    jointStateDim = -1;
//    baseJointStateDimMinusOne = -1;
//    baseDOFminusOne = -1;
//    if (rootLink->child_links[0]->parent_joint->type == urdf::Joint::FIXED)
//      rootLink = rootLink->child_links[0];
//  }
//
//  child_.emplace_back();
//  processLinkFromUrdf(rootLink, child_.back(), jointsNames_);
//  init();
}

} // object
} // bullet_sim
