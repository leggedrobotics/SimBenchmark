//
// Created by kangd on 15.02.18.
//

#include "DartSim.hpp"

#include "666Benchmark.hpp"
#include "DartBenchmark.hpp"

dart_sim::DartSim *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::sixsixsix::options.gui)
    sim = new dart_sim::DartSim(800, 600, 0.5,
                                     benchmark::NO_BACKGROUND,
                                     benchmark::dart::options.solverOption);
  else
    sim = new dart_sim::DartSim(benchmark::dart::options.solverOption);

  // timestep and max contact
  sim->setTimeStep(benchmark::sixsixsix::options.dt);
  sim->setMaxContacts(10000);

  /// no erp for dart
  if(benchmark::sixsixsix::options.erpYN)
  RAIFATAL("erp is not supported for dart")
}

void resetWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::sixsixsix::params.g});

  // random number generator
  rai::RandomNumberGenerator<double> rand;
  rand.seed(benchmark::sixsixsix::params.randomSeed);

  auto checkerboard = sim->addCheckerboard(5.0, 200.0, 200.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0);
  if(benchmark::sixsixsix::options.elasticCollision)
    checkerboard->setRestitutionCoefficient(1.0);

  for(int i = 0; i < benchmark::sixsixsix::params.n; i++) {
    for(int j = 0; j < benchmark::sixsixsix::params.n; j++) {
      for(int k = 0; k < benchmark::sixsixsix::params.n; k++) {

        // add object
        auto obj = sim->addSphere(benchmark::sixsixsix::params.ballR,
                                  benchmark::sixsixsix::params.ballM);

        // set position
        double x =
            double(i) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation;
        double y =
            double(j) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation;
        double z =
            double(k) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation
                + benchmark::sixsixsix::params.H;

        obj->setPosition(x, y, z);
        obj->setFrictionCoefficient(0);
        if(benchmark::sixsixsix::options.elasticCollision)
          obj->setRestitutionCoefficient(1.0);
//        obj->setOrientationRandom();

        if(benchmark::sixsixsix::options.gui) {
          if((i + j + k) % 3 == 0) {
            obj.visual()[0]->setColor({benchmark::dart::color[0],
                                       benchmark::dart::color[1],
                                       benchmark::dart::color[2]});
          }
          else if((i + j + k) % 3 == 1) {
            obj.visual()[0]->setColor({benchmark::dart::color[0],
                                       benchmark::dart::color[1],
                                       benchmark::dart::color[2]});
          }
          else if((i + j + k) % 3 == 2) {
            obj.visual()[0]->setColor({benchmark::dart::color[0],
                                       benchmark::dart::color[1],
                                       benchmark::dart::color[2]});
          }
        }

        objList.push_back(obj);
      }
    }
  }

  if(benchmark::sixsixsix::options.gui) {
    sim->setLightPosition((float)benchmark::sixsixsix::params.lightPosition[0],
                          (float)benchmark::sixsixsix::params.lightPosition[1],
                          (float)benchmark::sixsixsix::params.lightPosition[2]);
    sim->cameraFollowObject(objList[objList.size() / 2], {0, 5, 2});
  }
}

double penetrationCheck() {
  double error = 0;
  int numObj = objList.size();

  for (int i = 0; i < numObj; i++) {
    for (int j = i + 1; j < numObj; j++) {
      double dist = (objList[i]->getPosition() - objList[j]->getPosition()).norm();

      // error between spheres
      if (dist < benchmark::sixsixsix::params.ballR * 2)
        error += (benchmark::sixsixsix::params.ballR * 2 - dist) * (benchmark::sixsixsix::params.ballR * 2 - dist);
    }

    // error sphere ~ ground
    if (objList[i]->getPosition()[2] < benchmark::sixsixsix::params.ballR) {
      error +=
          (benchmark::sixsixsix::params.ballR - objList[i]->getPosition()[2]) * (benchmark::sixsixsix::params.ballR - objList[i]->getPosition()[2]);
    }
  }

  return error;
}

double computeEnergy() {
  double energy = 0;
  for(int j = 0; j < objList.size(); j++)
    energy += objList[j]->getEnergy({0, 0, benchmark::sixsixsix::params.g});
  return energy;
}

void simulationLoop() {

  // init
  double E0 = 0;
  benchmark::sixsixsix::energyList.reserve(unsigned(benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt));
  benchmark::sixsixsix::errorList.reserve(unsigned(benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt));

  // loop
  StopWatch watch;
  watch.start();
  if(benchmark::sixsixsix::options.gui) {
    // gui
    if(benchmark::sixsixsix::options.saveVideo)
      sim->startRecordingVideo("/tmp", "dart-thousand");

    for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt)
        && sim->visualizerLoop(benchmark::sixsixsix::options.dt); i++) {

      double error = penetrationCheck();
      double energy = computeEnergy();
      if(i==0) E0 = energy;

      benchmark::sixsixsix::energyList.push_back(energy);
      benchmark::sixsixsix::errorList.push_back(error);

      sim->integrate();
    }

    if(benchmark::sixsixsix::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt); i++) {

      double error = penetrationCheck();
      double energy = computeEnergy();
      if(i==0) E0 = energy;

      benchmark::sixsixsix::energyList.push_back(energy);
      benchmark::sixsixsix::errorList.push_back(error);

      sim->integrate();
    }
  }
  double time = watch.measure();

  benchmark::sixsixsix::printError(E0, time);
  if(benchmark::sixsixsix::options.csv)
    benchmark::sixsixsix::printCSV(benchmark::sixsixsix::getCSVpath(),
                                   benchmark::dart::options.simName,
                                   benchmark::dart::options.solverName,
                                   benchmark::dart::options.detectorName,
                                   benchmark::dart::options.integratorName,
                                   time,
                                   E0);
}

int main(int argc, const char* argv[]) {

  benchmark::sixsixsix::addDescToOption(desc);
  benchmark::dart::addDescToOption(desc);

  benchmark::sixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::dart::getOptionsFromArg(argc, argv, desc);

  benchmark::sixsixsix::getParamsFromYAML(benchmark::sixsixsix::getYamlpath().c_str(),
                                         benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: DART" << std::endl
                << "GUI      : " << benchmark::sixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::sixsixsix::options.erpYN << std::endl
                << "Timestep : " << benchmark::sixsixsix::options.dt << std::endl
                << "Solver   : " << benchmark::dart::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  resetWorld();
  simulationLoop();

  delete sim;
  return 0;

}
