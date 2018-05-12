//
// Created by kangd on 09.05.18.
//

#include <OdeWorld_RG.hpp>

#include "BuildingBenchmark.hpp"
#include "OdeBenchmark.hpp"

ode_sim::OdeWorld_RG *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::building::options.gui)
    sim = new ode_sim::OdeWorld_RG(800, 600, 0.05,
                                   benchmark::NO_BACKGROUND,
                                   benchmark::ode::options.solverOption);
  else
    sim = new ode_sim::OdeWorld_RG(benchmark::ode::options.solverOption);

  // erp
  if(benchmark::building::options.erpYN)
    sim->setERP(benchmark::building::params.erp, 0, 0);
  else
    sim->setERP(0, 0, 0);

  // set up logger and timer
  if(benchmark::building::options.log)
    benchmark::building::loggerSetup(
        benchmark::building::getLogDirpath(benchmark::building::options.erpYN,
                                           benchmark::ode::options.simName,
                                           benchmark::ode::options.solverName,
                                           benchmark::building::options.dt), "var"
    );
}

void setupWorld() {
  // add objects
  auto checkerboard = sim->addCheckerboard(10.0, 400.0, 400.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

  // block size
  const float shortLen = benchmark::building::params.shortLen;
  const float longLen = benchmark::building::params.longLen;
  const float heightLen = benchmark::building::params.heightLen;

  // num of blocks
  // numFloor x numBase + numFloor x (numWall x 2 + 1)
  const int numFloor = benchmark::building::params.numFloor;
  const int numBase = benchmark::building::params.numBase;
  const int numWall = numBase / 2;

  for(int i = 0; i < numFloor; i++) {
    // i floor
    for(int j = 0; j < numBase; j++) {
      // base
      auto base = sim->addBox(shortLen, longLen + 0.05, heightLen, 10.0);
      base->setPosition(j * longLen, 0, i * heightLen * 2 + 0.05);
      objList.push_back(base);

      if(benchmark::building::options.gui)
        base.visual()[0]->setColor({0.0, 0.0, 1.0});
    }

    for(int j = 0; j < numWall; j++) {
      // right wall
      auto wall = sim->addBox(longLen, shortLen, heightLen, 10.0);
      wall->setPosition(j * longLen * 2 + 0.1, -0.5 * longLen, i * heightLen * 2 + 0.15);
      objList.push_back(wall);

      if(benchmark::building::options.gui)
        wall.visual()[0]->setColor({0.0, 1.0, 0.0});
    }

    for(int j = 0; j < numWall - 1; j++) {
      // left wall
      auto wall = sim->addBox(longLen, shortLen, heightLen, 10.0);
      wall->setPosition(j * longLen * 2 + 0.3, 0.5 * longLen, i * heightLen * 2 + 0.15);
      objList.push_back(wall);

      if(benchmark::building::options.gui)
        wall.visual()[0]->setColor({1.0, 0.0, 0.0});
    }

    // first wall on left
    auto wall1 = sim->addBox(longLen, shortLen, heightLen, 10.0);
    wall1->setPosition(0.1, 0.5 * longLen, i * heightLen * 2 + 0.15);
    objList.push_back(wall1);

    // last wall on left
    auto wall2 = sim->addBox(longLen, shortLen, heightLen, 10.0);
    wall2->setPosition((numWall - 1) * longLen * 2 + 0.1, 0.5 * longLen, i * heightLen * 2 + 0.15);
    objList.push_back(wall2);

    if(benchmark::building::options.gui) {
      wall1.visual()[0]->setColor({1.0, 0.0, 0.0});
      wall2.visual()[0]->setColor({1.0, 0.0, 0.0});
    }
  }

  if(benchmark::building::options.gui) {
    sim->setLightPosition((float)benchmark::building::params.lightPosition[0],
                          (float)benchmark::building::params.lightPosition[1],
                          (float)benchmark::building::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {0, 5, 2});
  }
}

void simulationLoop() {
  if(benchmark::building::options.gui) {
    // gui
    double numContact = 0;
    int i = 0;

    if(benchmark::building::options.saveVideo)
      sim->startRecordingVideo("/tmp", "ode-building");

    while(sim->visualizerLoop(benchmark::building::options.dt)) {

      if(objList.back()->getPosition()[2] <
          benchmark::building::params.heightLen * (benchmark::building::params.numFloor - 1) * 2) {
        // break if the building collapses
        RAIINFO("building collapsed!")
        break;
      }

      sim->integrate(benchmark::building::options.dt);

      // calculate average contacts
      numContact = double(i) / double(i+1) * numContact + sim->getWorldNumContacts() / double(i+1);
      i++;
    }

    if(benchmark::building::options.saveVideo)
      sim->stopRecordingVideo();

    std::cout << "average contact " << numContact << "\n";
  }
  else {
    // no gui
    double numContact = 0;

    StopWatch watch;
    watch.start();

    int i = 0;
    for(i = 0; i < (int) (benchmark::building::options.T / benchmark::building::options.dt); i++) {

      if(objList.back()->getPosition()[2] <
          benchmark::building::params.heightLen * (benchmark::building::params.numFloor - 1) * 2) {
        // break if the building collapses
        RAIINFO("building collapsed!")
        break;
      }

      sim->integrate(benchmark::building::options.dt);

      // calculate average contacts
      numContact = double(i) / double(i+1) * numContact + sim->getWorldNumContacts() / double(i+1);
    }

    // print to screen
    double time = watch.measure();
    std::cout << "time taken for " << i << " steps "<< time <<"s \n";
    std::cout << "average contact " << numContact << "\n";
  }
}

int main(int argc, const char* argv[]) {

  benchmark::building::addDescToOption(desc);
  benchmark::ode::addDescToOption(desc);

  benchmark::building::getOptionsFromArg(argc, argv, desc);
  benchmark::ode::getOptionsFromArg(argc, argv, desc);

  benchmark::building::getParamsFromYAML(benchmark::building::getYamlpath().c_str(),
                                         benchmark::ODE);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: ODE" << std::endl
                << "GUI      : " << benchmark::building::options.gui << std::endl
                << "ERP      : " << benchmark::building::options.erpYN << std::endl
                << "Timestep : " << benchmark::building::options.dt << std::endl
                << "Num block: " << objList.size() << std::endl
                << "Solver   : " << benchmark::ode::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  // time log
//  if(benchmark::building::options.log)
//    ru::timer->dumpToStdOuput();

  delete sim;
  return 0;
}
