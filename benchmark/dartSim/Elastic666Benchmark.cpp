//
// Created by kangd on 15.02.18.
//

#include "DartSim.hpp"

#include "Elastic666Benchmark.hpp"
#include "DartBenchmark.hpp"

dart_sim::DartSim *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

double computeEnergy() {
  double energy = 0;
  for(int j = 0; j < objList.size(); j++)
    energy += objList[j]->getEnergy({0, 0, benchmark::elasticsixsixsix::params.g});
  return energy;
}

void setupSimulation() {
  if (benchmark::elasticsixsixsix::options.gui)
    sim = new dart_sim::DartSim(800, 600, 0.5,
                                benchmark::NO_BACKGROUND,
                                benchmark::dart::options.solverOption);
  else
    sim = new dart_sim::DartSim(benchmark::dart::options.solverOption);

  // timestep and max contact
  sim->setTimeStep(benchmark::elasticsixsixsix::options.dt);
  sim->setMaxContacts(10000);

  /// no erp for dart
  if(benchmark::elasticsixsixsix::options.erpYN)
  RAIFATAL("erp is not supported for dart")
}

void setupWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::elasticsixsixsix::params.g});

  // random number generator
  rai::RandomNumberGenerator<double> rand;
  rand.seed(benchmark::elasticsixsixsix::params.randomSeed);

  auto checkerboard = sim->addCheckerboard(5.0, 200.0, 200.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0);
  checkerboard->setRestitutionCoefficient(1.0);

  for(int i = 0; i < benchmark::elasticsixsixsix::params.n; i++) {
    for(int j = 0; j < benchmark::elasticsixsixsix::params.n; j++) {
      for(int k = 0; k < benchmark::elasticsixsixsix::params.n; k++) {

        // add object
        auto obj = sim->addSphere(benchmark::elasticsixsixsix::params.ballR,
                                  benchmark::elasticsixsixsix::params.ballM);

        // set position
        double x =
            double(i) * benchmark::elasticsixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation;
        double y =
            double(j) * benchmark::elasticsixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation;
        double z =
            double(k) * benchmark::elasticsixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation
                + benchmark::elasticsixsixsix::params.H;

        obj->setPosition(x, y, z);
        obj->setFrictionCoefficient(0);
        obj->setRestitutionCoefficient(1.0);

        if(benchmark::elasticsixsixsix::options.gui) {
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

  if(benchmark::elasticsixsixsix::options.gui) {
    sim->setLightPosition((float)benchmark::elasticsixsixsix::params.lightPosition[0],
                          (float)benchmark::elasticsixsixsix::params.lightPosition[1],
                          (float)benchmark::elasticsixsixsix::params.lightPosition[2]);
    sim->cameraFollowObject(objList[objList.size() / 2], {0, 5, 2});
  }
}

double simulationLoop(bool timer = true, bool error = true) {
  // gui
  if(benchmark::elasticsixsixsix::options.gui && benchmark::elasticsixsixsix::options.saveVideo)
    sim->startRecordingVideo("/tmp", "dart-666");

  // resever error vector
  benchmark::elasticsixsixsix::data.setN(unsigned(benchmark::elasticsixsixsix::options.T / benchmark::elasticsixsixsix::options.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  for(int i = 0; i < (int) (benchmark::elasticsixsixsix::options.T / benchmark::elasticsixsixsix::options.dt); i++) {
    if (benchmark::elasticsixsixsix::options.gui && !sim->visualizerLoop(benchmark::elasticsixsixsix::options.dt))
      break;

    // data save
    if (error) {
      static double E0 = 0;
      if(i==0)
        E0 = computeEnergy();

      double error = pow(computeEnergy() - E0, 2);
      benchmark::elasticsixsixsix::data.error.push_back(error);
    }

    sim->integrate();
  }

  if(benchmark::elasticsixsixsix::options.saveVideo)
    sim->stopRecordingVideo();

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::elasticsixsixsix::addDescToOption(desc);
  benchmark::dart::addDescToOption(desc);

  benchmark::elasticsixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::dart::getOptionsFromArg(argc, argv, desc);

  benchmark::elasticsixsixsix::getParamsFromYAML(benchmark::elasticsixsixsix::getYamlpath().c_str(),
                                                 benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: " << benchmark::dart::options.simName << std::endl
                << "GUI      : " << benchmark::elasticsixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::elasticsixsixsix::options.erpYN << std::endl
                << "Timestep : " << benchmark::elasticsixsixsix::options.dt << std::endl
                << "Solver   : " << benchmark::dart::options.solverName << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::elasticsixsixsix::data.computeError();

  // reset
  objList.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);


  if(benchmark::elasticsixsixsix::options.csv)
    benchmark::elasticsixsixsix::printCSV(benchmark::elasticsixsixsix::getCSVpath(),
                                          benchmark::dart::options.simName,
                                          benchmark::dart::options.solverName,
                                          benchmark::dart::options.detectorName,
                                          benchmark::dart::options.integratorName,
                                          time,
                                          error);

  RAIINFO(
      std::endl << "CPU time   : " << time << std::endl
                << "mean error : " << error << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;

}
