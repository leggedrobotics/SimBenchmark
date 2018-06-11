//
// Created by kangd on 09.05.18.
//

#include <BtMbSim.hpp>

#include "BuildingBenchmark.hpp"
#include "BtMbBenchmark.hpp"

bullet_mb_sim::BtMbSim *sim;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::building::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.05,
                                     benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // time step
  sim->setTimeStep(benchmark::building::options.dt);


  // erp
  if(benchmark::building::options.erpYN)
    sim->setERP(benchmark::building::params.erp,
                benchmark::building::params.erp2,
                benchmark::building::params.erpFriction);
  else
    sim->setERP(0, 0, 0);

  // set up logger and timer
  if(benchmark::building::options.log)
    benchmark::building::loggerSetup(
        benchmark::building::getLogDirpath(benchmark::building::options.erpYN,
                                           benchmark::bulletmultibody::options.simName,
                                           benchmark::bulletmultibody::options.solverName,
                                           benchmark::building::options.dt), "var"
    );
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addArticulatedSystem(
      benchmark::building::getBulletPlanePath(),
      bullet_mb_sim::object::URDF
  );

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
      auto base = sim->addArticulatedSystem(
          benchmark::building::getBulletBasePath(),
          bullet_mb_sim::object::URDF
      );
      base->setGeneralizedCoordinate(
          {j * longLen,
           0,
           i * heightLen * 2 + 0.05,
           1, 0, 0, 0});
      objList.push_back(base);

      if(benchmark::building::options.gui)
        base.visual()[0]->setColor({0.0, 0.0, 1.0});
    }

    for(int j = 0; j < numWall; j++) {
      // right wall
      auto wall = sim->addArticulatedSystem(
          benchmark::building::getBulletWallPath(),
          bullet_mb_sim::object::URDF
      );
      wall->setGeneralizedCoordinate(
          {j * longLen * 2 + 0.1,
           -0.5 * longLen,
           i * heightLen * 2 + 0.15,
           1, 0, 0, 0
          });
      objList.push_back(wall);

      if(benchmark::building::options.gui)
        wall.visual()[0]->setColor({0.0, 1.0, 0.0});
    }

    for(int j = 0; j < numWall - 1; j++) {
      // left wall
      auto wall = sim->addArticulatedSystem(
          benchmark::building::getBulletWallPath(),
          bullet_mb_sim::object::URDF
      );
      wall->setGeneralizedCoordinate(
          {j * longLen * 2 + 0.3,
           0.5 * longLen,
           i * heightLen * 2 + 0.15,
           1, 0, 0, 0});
      objList.push_back(wall);

      if(benchmark::building::options.gui)
        wall.visual()[0]->setColor({1.0, 0.0, 0.0});
    }

    // first wall on left
    auto wall1 = sim->addArticulatedSystem(
        benchmark::building::getBulletWallPath(),
        bullet_mb_sim::object::URDF
    );
    wall1->setGeneralizedCoordinate(
        {0.1,
         0.5 * longLen,
         i * heightLen * 2 + 0.15,
         1, 0, 0, 0});
    objList.push_back(wall1);

    // last wall on left
    auto wall2 = sim->addArticulatedSystem(
        benchmark::building::getBulletWallPath(),
        bullet_mb_sim::object::URDF
    );
    wall2->setGeneralizedCoordinate(
        {(numWall - 1) * longLen * 2 + 0.1,
         0.5 * longLen,
         i * heightLen * 2 + 0.15,
         1, 0, 0, 0});
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

benchmark::building::Data simulationLoop() {
  if(benchmark::building::options.saveVideo)
    sim->startRecordingVideo("/tmp", "rai-building");

  // data
  benchmark::building::Data data;
  data.setN(unsigned(benchmark::building::options.T / benchmark::building::options.dt));

  // timer start
  StopWatch watch;
  watch.start();

  int cnt = (int) (benchmark::building::options.T / benchmark::building::options.dt);
  for(int i = 0; i < (int) (benchmark::building::options.T / benchmark::building::options.dt); i++) {
    // gui
    if (benchmark::building::options.gui && !sim->visualizerLoop(benchmark::building::options.dt))
      break;

    // num contacts
    data.numContacts.push_back(sim->getWorldNumContacts());

    if(objList.back()->getGeneralizedCoordinate()[2] <
        benchmark::building::params.heightLen * (benchmark::building::params.numFloor - 1) * 2) {
      // break if the building collapses
      cnt = i+1;
      RAIINFO("building collapsed after " << cnt << " steps = " << cnt * benchmark::building::options.dt << " sec!")
      break;
    }

    sim->integrate();
  }

  if(benchmark::building::options.saveVideo)
    sim->stopRecordingVideo();

  data.time = watch.measure();
  data.step = cnt;
  return data;
}

int main(int argc, const char* argv[]) {

  benchmark::building::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::building::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::building::getParamsFromYAML(benchmark::building::getYamlpath().c_str(),
                                         benchmark::DART);

  setupSimulation();
  setupWorld();

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: " << benchmark::bulletmultibody::options.simName << std::endl
                << "GUI      : " << benchmark::building::options.gui << std::endl
                << "ERP      : " << benchmark::building::options.erpYN << std::endl
                << "Timestep : " << benchmark::building::options.dt << std::endl
                << "Num block: " << objList.size() << std::endl
                << "-----------------------"
  )

  benchmark::building::Data data = simulationLoop();

  if(benchmark::building::options.csv)
    benchmark::building::printCSV(benchmark::building::getCSVpath(),
                                  benchmark::bulletmultibody::options.simName,
                                  benchmark::bulletmultibody::options.solverName,
                                  benchmark::bulletmultibody::options.detectorName,
                                  benchmark::bulletmultibody::options.integratorName,
                                  data.time,
                                  data.step,
                                  data.computeMeanContacts());

  RAIINFO(
      std::endl << "Avg. Num Contacts : " << data.computeMeanContacts() << std::endl
                << "CPU time          : " << data.time << std::endl
                << "num steps         : " << data.step << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;
}
