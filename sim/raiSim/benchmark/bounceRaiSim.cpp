//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "bounce.hpp"

int main(int argc, char* argv[]) {

  double dt = benchmark::dt;
  double restitution = benchmark::restitution;

  if (argc == 2) {
    dt = atof(argv[1]);
    RAIINFO("----------------------")
    RAIINFO("raiSim")
    RAIINFO("timestep = " << dt);
  } else if (argc == 3) {
    dt = atof(argv[1]);

    restitution = atof(argv[2]);

    RAIINFO("----------------------")
    RAIINFO("BulletSim")
    RAIINFO("timestep    = " << dt);
    RAIINFO("restitution = " << restitution);
  }

  // logger
  std::string path = benchmark::dataPath + benchmark::parentDir + "rai";
  std::string name = std::to_string(dt) + std::to_string(restitution);
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->setLogFileName(name);
  rai::Utils::logger->setOptions(rai::Utils::ONEFILE_FOR_ONEDATA);
  rai::Utils::logger->addVariableToLog(3, "velball", "linear velocity of ball");
  rai::Utils::logger->addVariableToLog(3, "posball", "position of ball");

  // timer
  std::string timer = name + "timer";
  rai::Utils::timer->setLogPath(path);
  rai::Utils::timer->setLogFileName(timer);

  // sim
  rai_sim::World_RG *sim;

  if(benchmark::visualize)
    sim = new rai_sim::World_RG(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG;

  rai_sim::MaterialManager materials;

  sim->setGravity(benchmark::gravity);

  // solver params
  sim->setContactSolverParam(1.0, 0.7, 1.0, 50, 1e-12);
  sim->setERP(benchmark::erp);

  // materials
  materials.setMaterialNames({"ground", "ball"});
  materials.setMaterialPairProp("ground", "ball",
                                benchmark::friction,
                                restitution * restitution,
                                0.01);
  sim->updateMaterialProp(materials);

  // add objects
  // checkerboard
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setMaterial(sim->getMaterialKey("ground"));

  // ball
  std::vector<rai_sim::SingleBodyHandle> objectList;

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, benchmark::dropHeight);
      ball->setMaterial(sim->getMaterialKey("ball"));
      objectList.push_back(ball);
    }
  }

  // simulation loop
  // press 'q' key to quit
  rai::Utils::timer->startTimer("bounce");
  if(benchmark::visualize) {
    // camera relative position
    sim->setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);
    sim->cameraFollowObject(checkerboard, {50, 0, 5});

    for(int i = 0; i < benchmark::simulationTime / dt && sim->visualizerLoop(dt); i++) {
      // log
      rai::Utils::logger->appendData("velball", objectList[12]->getLinearVelocity().data());
      rai::Utils::logger->appendData("posball", objectList[12]->getPosition().data());
      sim->integrate(dt);
    }
  }
  else {
    for(int i = 0; i < benchmark::simulationTime / dt; i++) {
      // log
      rai::Utils::logger->appendData("velball", objectList[12]->getLinearVelocity().data());
      rai::Utils::logger->appendData("posball", objectList[12]->getPosition().data());
      sim->integrate(dt);
    }
  }
  rai::Utils::timer->stopTimer("bounce");

  // delete sim
  delete sim;

  // time log
  rai::Utils::timer->dumpToStdOuput();

  return 0;
}