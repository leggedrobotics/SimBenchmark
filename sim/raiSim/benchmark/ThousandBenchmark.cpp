//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "ThousandBenchmark.hpp"

rai_sim::World_RG *sim;
std::vector<rai_sim::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::thousand::options.gui)
    sim = new rai_sim::World_RG(800, 600, 1, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG();

  // erp
  if(benchmark::thousand::options.erpYN)
    sim->setERP(benchmark::thousand::params.erp);
  else
    sim->setERP(0);

  // set up logger and timer
  if(benchmark::thousand::options.log)
    benchmark::thousand::loggerSetup(
        benchmark::thousand::getLogDirpath(benchmark::thousand::options.erpYN,
                                           "RAI",
                                           "RAI",
                                           benchmark::thousand::options.dt), "var"
    );
}

void setupWorld() {

  // random number generator
  rai::RandomNumberGenerator<double> rand;
  rand.seed(benchmark::thousand::params.randomSeed);

  auto checkerboard = sim->addCheckerboard(5.0, 200.0, 200.0, 0.1, 1, -1, rai_sim::GRID);

  for(int i = 0; i < benchmark::thousand::params.n; i++) {
    for(int j = 0; j < benchmark::thousand::params.n; j++) {
      for(int k = 0; k < benchmark::thousand::params.n; k++) {

        // add object
        auto obj = sim->addSphere(benchmark::thousand::params.ballR,
                                  benchmark::thousand::params.ballM);

        // set position
        double x =
            double(i) * benchmark::thousand::params.gap
                + rand.sampleUniform01() * benchmark::thousand::params.perturbation;
        double y =
            double(j) * benchmark::thousand::params.gap
                + rand.sampleUniform01() * benchmark::thousand::params.perturbation;
        double z =
            double(k) * benchmark::thousand::params.gap
                + rand.sampleUniform01() * benchmark::thousand::params.perturbation
                + benchmark::thousand::params.H;

        obj->setPosition(x, y, z);
//        obj->setOrientationRandom();

        if(benchmark::thousand::options.gui) {
          if((i + j + k) % 3 == 0) {
            obj.visual()[0]->setColor({0.5373,
                                       0.6471,
                                       0.3059});
          }
          else if((i + j + k) % 3 == 1) {
            obj.visual()[0]->setColor({0.5373,
                                       0.6471,
                                       0.3059});
          }
          else if((i + j + k) % 3 == 2) {
            obj.visual()[0]->setColor({0.5373,
                                       0.6471,
                                       0.3059});
          }
        }

        objList.push_back(obj);
      }
    }
  }

  if(benchmark::thousand::options.gui) {
    sim->setLightPosition((float)benchmark::thousand::params.lightPosition[0],
                          (float)benchmark::thousand::params.lightPosition[1],
                          (float)benchmark::thousand::params.lightPosition[2]);
    sim->cameraFollowObject(objList[objList.size() / 2], {0, 20, 10});
  }
}

double penetrationCheck() {
  double error = 0;
  int numObj = objList.size();

  for (int i = 0; i < numObj; i++) {
    for (int j = i + 1; j < numObj; j++) {
      double dist = (objList[i]->getPosition() - objList[j]->getPosition()).norm();

      // error between spheres
      if (dist < benchmark::thousand::params.ballR * 2)
        error += (benchmark::thousand::params.ballR * 2 - dist) * (benchmark::thousand::params.ballR * 2 - dist);
    }

    // error sphere ~ ground
    if (objList[i]->getPosition()[2] < benchmark::thousand::params.ballR) {
      error +=
          (benchmark::thousand::params.ballR - objList[i]->getPosition()[2]) * (benchmark::thousand::params.ballR - objList[i]->getPosition()[2]);
    }
  }

  return error;
}

void simulationLoop() {
  if(benchmark::thousand::options.timer)
    rai::Utils::timer->startTimer("thousand");

  if(benchmark::thousand::options.gui) {
    // gui
    if(benchmark::thousand::options.saveVideo)
      sim->startRecordingVideo("/tmp", "rai-building");

    for(int i = 0; i < (int) (benchmark::thousand::options.T / benchmark::thousand::options.dt)
        && sim->visualizerLoop(benchmark::thousand::options.dt); i++) {

      sim->integrate(benchmark::thousand::options.dt);

      if(benchmark::thousand::options.log) {
        double error = penetrationCheck();
        ru::logger->appendData("error", error);
      }
    }

    if(benchmark::thousand::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    for(int i = 0; i < (int) (benchmark::thousand::options.T / benchmark::thousand::options.dt); i++) {

      sim->integrate(benchmark::thousand::options.dt);

      if(benchmark::thousand::options.log) {
        double error = penetrationCheck();
        ru::logger->appendData("error", error);
      }
    }
  }

  if (benchmark::thousand::options.timer)
    rai::Utils::timer->stopTimer("thousand");
}

int main(int argc, const char* argv[]) {

  benchmark::thousand::addDescToOption(desc);
  benchmark::thousand::getOptionsFromArg(argc, argv, desc);
  benchmark::thousand::getParamsFromYAML(benchmark::thousand::getYamlpath().c_str(),
                                         benchmark::RAI);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::thousand::options.gui << std::endl
                << "ERP      : " << benchmark::thousand::options.erpYN << std::endl
                << "Timestep : " << benchmark::thousand::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  // time log
  if(benchmark::thousand::options.timer)
    ru::timer->dumpToStdOuput();

  delete sim;
  return 0;

}
