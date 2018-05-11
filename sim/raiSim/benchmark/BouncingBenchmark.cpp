//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "BouncingBenchmark.hpp"

rai_sim::World_RG *sim;
std::vector<rai_sim::SingleBodyHandle> ballList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::bouncing::options.gui)
    sim = new rai_sim::World_RG(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG();

  // erp
  if(benchmark::bouncing::options.erpYN)
    sim->setERP(benchmark::bouncing::params.erp);
  else
    sim->setERP(0);

  // set up logger and timer
  if(benchmark::bouncing::options.log)
    benchmark::bouncing::loggerSetup(
        benchmark::bouncing::getLogDirpath(benchmark::bouncing::options.erpYN,
                                           benchmark::bouncing::options.e,
                                           "RAI",
                                           "RAI",
                                           benchmark::bouncing::options.dt), "var"
    );
}

void setupWorld() {
  // materials
  rai_sim::MaterialManager materials;
  materials.setMaterialNames({"ground", "ball"});
  materials.setMaterialPairProp("ground", "ball",
                                benchmark::bouncing::params.mu_ground * benchmark::bouncing::params.mu_ball,
                                benchmark::bouncing::options.e * benchmark::bouncing::options.e,
                                0.01);
  sim->updateMaterialProp(materials);

  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, 1, -1, rai_sim::GRID);
  checkerboard->setMaterial(sim->getMaterialKey("ground"));

  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0, j * 2.0, benchmark::bouncing::params.H);
      ball->setMaterial(sim->getMaterialKey("ball"));
      ball.visual()[0]->setColor({0.5373, 0.6471, 0.3059});
      ballList.push_back(ball);
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::bouncing::params.g});

  if(benchmark::bouncing::options.gui) {
    sim->setLightPosition((float)benchmark::bouncing::params.lightPosition[0],
                          (float)benchmark::bouncing::params.lightPosition[1],
                          (float)benchmark::bouncing::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {30, 0, 15});
  }
}

void simulationLoop() {
  if(benchmark::bouncing::options.gui) {
    // gui
    if(benchmark::bouncing::options.saveVideo)
      sim->startRecordingVideo("/tmp", "rai-rolling");

    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt)
        && sim->visualizerLoop(benchmark::bouncing::options.dt); i++) {
      sim->integrate(benchmark::bouncing::options.dt);

      // energy log
      if(benchmark::bouncing::options.log) {
        double energy = 0;
        for(int j = 0; j < ballList.size(); j++) {
          energy += ballList[j]->getEnergy();
        }
        rai::Utils::logger->appendData("energy", energy);
      }
    }

    if(benchmark::bouncing::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt); i++) {
      sim->integrate(benchmark::bouncing::options.dt);

      if(benchmark::bouncing::options.log) {
        double energy = 0;
        for(int j = 0; j < ballList.size(); j++) {
          energy += ballList[j]->getEnergy();
        }
        rai::Utils::logger->appendData("energy", energy);
      }
    }
  }
}

int main(int argc, const char* argv[]) {

  benchmark::bouncing::addDescToOption(desc);
  benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
  benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlpath().c_str(),
                                         benchmark::RAI);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                << "Timestep : " << benchmark::bouncing::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  // time log
//  if(benchmark::bouncing::options.log)
//    ru::timer->dumpToStdOuput();

  delete sim;
  return 0;
}