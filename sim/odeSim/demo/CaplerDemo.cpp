//
// Created by jhwangbo on 04/12/17.
//
#include <OdeWorld_RG.hpp>

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/Capler/";

  ode_sim::OdeWorld_RG sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  sim.setLightPosition(0, -10, 10);

  auto checkerBoard = sim.addCheckerboard(1, 100, 100, 0.1,
                                          benchmark::object::PLANE_SHAPE,
                                          1, -1,
                                          benchmark::object::GRID);
  auto capler = sim.addArticulatedSystem(urdfPath);

  Eigen::VectorXd gc(capler->getStateDimension());
  Eigen::VectorXd gv(capler->getDOF()), tau(capler->getDOF());

  gc << 1.0, 0.0, 0.0;
  gv.setZero();
  tau.setZero();

//  capler->setState(gc, gv);
//  capler->setGeneralizedForce(tau);

  sim.cameraFollowObject(checkerBoard, {2, 2, 2});

  sim.loop(0.001);
}