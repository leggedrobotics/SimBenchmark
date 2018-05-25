//
// Created by kangd on 15.02.18.
//

#include <BtSim.hpp>

#include "BouncingBenchmark.hpp"
#include "BtBenchmark.hpp"

bullet_sim::BtSim *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::bouncing::options.gui)
    sim = new bullet_sim::BtSim(800, 600, 0.5, benchmark::bullet::options.solverOption, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_sim::BtSim(benchmark::bullet::options.solverOption);


  // erp
  if(benchmark::bouncing::options.erpYN)
    sim->setERP(benchmark::bouncing::params.erp, 0, 0);
  else
    sim->setERP(0, 0, 0);

  // set up logger and timer
  if(benchmark::bouncing::options.log)
    benchmark::bouncing::loggerSetup(
        benchmark::bouncing::getLogDirpath(benchmark::bouncing::options.erpYN,
                                           benchmark::bouncing::options.e,
                                           benchmark::bullet::options.simName,
                                           benchmark::bullet::options.solverName,
                                           benchmark::bouncing::options.dt), "var"
    );
}

void setupWorld() {
  // materials
  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(benchmark::bouncing::params.mu_ground);
  checkerboard->setRestitutionCoefficient(1.0);

  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      auto ball = sim->addSphere(benchmark::bouncing::params.R, benchmark::bouncing::params.m);
      ball->setPosition(i * 2.0 - 10, j * 2.0 - 10, benchmark::bouncing::params.H);
      ball->setFrictionCoefficient(benchmark::bouncing::params.mu_ball);
      ball->setRestitutionCoefficient(benchmark::bouncing::options.e);

      if(benchmark::bouncing::options.gui)
        ball.visual()[0]->setColor(
            {benchmark::bullet::color[0],
             benchmark::bullet::color[1],
             benchmark::bullet::color[2]});

      objList.push_back(ball);
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::bouncing::params.g});

  if(benchmark::bouncing::options.gui) {
    sim->setLightPosition((float)benchmark::bouncing::params.lightPosition[0],
                          (float)benchmark::bouncing::params.lightPosition[1],
                          (float)benchmark::bouncing::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {10, 0, 6});
  }
}

void simulationLoop() {
  if(benchmark::bouncing::options.gui) {
    // gui
    if(benchmark::bouncing::options.saveVideo)
      sim->startRecordingVideo("/tmp", "bullet-bouncing");

    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt)
        && sim->visualizerLoop(benchmark::bouncing::options.dt); i++) {
      sim->integrate(benchmark::bouncing::options.dt);

      // energy log
      if(benchmark::bouncing::options.log) {
        double energy = 0;
        for(int j = 0; j < objList.size(); j++) {
          energy += objList[j]->getEnergy({0, 0, benchmark::bouncing::params.g});
        }
        rai::Utils::logger->appendData("energy", energy);
      }
    }

    if(benchmark::bouncing::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    if(benchmark::bouncing::options.log)
      ru::timer->startTimer("bouncing");

    for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt); i++) {
      sim->integrate(benchmark::bouncing::options.dt);

      if(benchmark::bouncing::options.log) {
        double energy = 0;
        for(int j = 0; j < objList.size(); j++) {
          energy += objList[j]->getEnergy({0, 0, benchmark::bouncing::params.g});
        }
        rai::Utils::logger->appendData("energy", energy);
      }
    }

    if(benchmark::bouncing::options.log)
      ru::timer->stopTimer("bouncing");
  }
}

int main(int argc, const char* argv[]) {

  benchmark::bouncing::addDescToOption(desc);
  benchmark::bullet::addDescToOption(desc);

  benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
  benchmark::bullet::getOptionsFromArg(argc, argv, desc);

  benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlpath().c_str(),
                                         benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: BULLET" << std::endl
                << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                << "Timestep : " << benchmark::bouncing::options.dt << std::endl
                << "Solver   : " << benchmark::bullet::options.solverName << std::endl
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