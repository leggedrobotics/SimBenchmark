//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "rolling.hpp"

int main(int argc, char* argv[]) {

  double dt = benchmark::dt;
  if (argc == 2) {
    dt = atof(argv[1]);
    RAIINFO("----------------------")
    RAIINFO("raiSim")
    RAIINFO("timestep = " << dt);
  }

  // logger
  std::string path = benchmark::dataPath + benchmark::parentDir + "rai";
  std::string name = std::to_string(dt);
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->setLogFileName(name);
  rai::Utils::logger->setOptions(rai::Utils::ONEFILE_FOR_ONEDATA);
  rai::Utils::logger->addVariableToLog(3, "velbox", "linear velocity of box");
  rai::Utils::logger->addVariableToLog(3, "velball", "linear velocity of ball");
  rai::Utils::logger->addVariableToLog(3, "posbox", "position of box");
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

  // materials
  materials.setMaterialNames({"ground", "box", "ball"});
  materials.setMaterialPairProp("ground", "ball", benchmark::groundMu * benchmark::ballMu, 0.0, 0.01);
  materials.setMaterialPairProp("ground", "box", benchmark::groundMu * benchmark::boxMu, 0.0, 0.01);
  materials.setMaterialPairProp("ball", "box", benchmark::ballMu * benchmark::boxMu, 0.0, 0.01);
  sim->updateMaterialProp(materials);

  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setMaterial(sim->getMaterialKey("ground"));

  auto box = sim->addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5 - benchmark::initPenetration);
  box->setMaterial(sim->getMaterialKey("box"));

  std::vector<rai_sim::SingleBodyHandle> objectList;

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5 - 2 * benchmark::initPenetration);
      ball->setMaterial(sim->getMaterialKey("ball"));
      objectList.push_back(ball);
    }
  }

  // simulation loop
  // press 'q' key to quit
  rai_sim::Vec<3> force = {benchmark::force[0], benchmark::force[1], benchmark::force[2]};

  rai::Utils::timer->startTimer("rolling");
  if(benchmark::visualize) {
    // camera relative position
    sim->cameraFollowObject(box, {30, 0, 10});
    sim->setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);

    for(int i = 0; i < benchmark::simulationTime / dt && sim->visualizerLoop(dt); i++) {
      box->setExternalForce(force, 0);
      // log
      rai::Utils::logger->appendData("velbox", box->getLinearVelocity().data());
      rai::Utils::logger->appendData("velball", objectList[0]->getLinearVelocity().data());
      rai::Utils::logger->appendData("posbox", box->getPosition().data());
      rai::Utils::logger->appendData("posball", objectList[0]->getPosition().data());
      sim->integrate(dt);
    }
  } else {
    for(int i = 0; i < benchmark::simulationTime / dt; i++) {
      box->setExternalForce(force, 0);
      // log
      rai::Utils::logger->appendData("velbox", box->getLinearVelocity().data());
      rai::Utils::logger->appendData("velball", objectList[0]->getLinearVelocity().data());
      rai::Utils::logger->appendData("posbox", box->getPosition().data());
      rai::Utils::logger->appendData("posball", objectList[0]->getPosition().data());
      sim->integrate(dt);
    }
  }

  rai::Utils::timer->stopTimer("rolling");

  // delete sim
  delete sim;

  // time log
  rai::Utils::timer->dumpToStdOuput();

  return 0;
}