//
// Created by kangd on 26.04.18.
//

#include <MjcSim.hpp>

#include "AtlasContactBenchmark.hpp"
#include "MjcBenchmark.hpp"
#include "raiCommon/utils/StopWatch.hpp"

mujoco_sim::MjcSim *sim;
po::options_description desc;

void setupSimulation() {
  int numRow = benchmark::atlas::options.numRow;

  // gui
  if(benchmark::atlas::options.gui)
    sim = new mujoco_sim::MjcSim(800, 600, 0.5,
                                 benchmark::atlas::getMujocoURDFpath(numRow).c_str(),
                                 benchmark::mujoco::getKeypath().c_str(),
                                 benchmark::NO_BACKGROUND,
                                 benchmark::mujoco::options.solverOption,
                                 benchmark::mujoco::options.integratorOption);
  else
    sim = new mujoco_sim::MjcSim(benchmark::atlas::getMujocoURDFpath(numRow).c_str(),
                                 benchmark::mujoco::getKeypath().c_str(),
                                 benchmark::mujoco::options.solverOption,
                                 benchmark::mujoco::options.integratorOption);

  // time step
  sim->setTimeStep(benchmark::atlas::params.dt);
}

void resetWorld() {
  int numRow = benchmark::atlas::options.numRow;
  Eigen::VectorXd genCoord(sim->getStateDimension());
  genCoord.setZero();

  int cnt = 0;
  for(int i = 0; i < numRow; i++) {
    for (int j = 0; j < numRow; j++) {
      genCoord[cnt * 36 + 0] = i * 2.5;
      genCoord[cnt * 36 + 1] = j * 2.5;
      genCoord[cnt * 36 + 2] = benchmark::atlas::params.H;
      genCoord[cnt * 36 + 3] = benchmark::atlas::params.baseQuat[0];
      genCoord[cnt * 36 + 4] = benchmark::atlas::params.baseQuat[1];
      genCoord[cnt * 36 + 5] = benchmark::atlas::params.baseQuat[2];
      genCoord[cnt * 36 + 6] = benchmark::atlas::params.baseQuat[3];
      cnt++;
    }
  }

  sim->setGeneralizedCoordinate(genCoord);
  sim->setGeneralizedVelocity(Eigen::VectorXd::Zero(sim->getDOF()));
  sim->setGeneralizedForce(Eigen::VectorXd::Zero(sim->getDOF()));

  sim->setGravity({0, 0, benchmark::atlas::params.g});

  if(benchmark::atlas::options.gui)
    sim->cameraFollowObject(
        sim->getSingleBodyHandle(0), {1.0, 1.0, 1.0});
}

double simulationLoop() {
  int numContact = 0;
  int cnt = 0;
  if(benchmark::atlas::options.gui) {
    // gui
    while(sim->visualizerLoop(benchmark::atlas::params.dt)) {
      sim->integrate();
      numContact += sim->getWorldNumContacts();
      cnt++;
    }
  } else {
    // no gui
    StopWatch watch;
    watch.start();
    for(int t = 0; t < (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt); t++) {
      sim->integrate();
      numContact += sim->getWorldNumContacts();
      cnt++;
    }

    double time = watch.measure();

    // print to screen
    std::cout<<"time taken for "
             << (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt)
             << " steps "<< time <<"s \n";

    if(benchmark::atlas::options.csv)
      benchmark::atlas::printCSV(benchmark::atlas::getCSVpath(),
                                 benchmark::mujoco::options.simName,
                                 benchmark::mujoco::options.solverName,
                                 benchmark::mujoco::options.detectorName,
                                 benchmark::mujoco::options.integratorName,
                                 benchmark::atlas::options.numRow,
                                 double(numContact) / cnt,
                                 time);
  }
  return double(numContact) / cnt;
}

int main(int argc, const char* argv[]) {

  benchmark::atlas::addDescToOption(desc);
  benchmark::atlas::getOptionsFromArg(argc, argv, desc);

  benchmark::atlas::getParamsFromYAML(benchmark::atlas::getYamlpath().c_str(),
                                      benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::atlas::options.gui << std::endl
                << "Row      : " << benchmark::atlas::options.numRow << std::endl
                << "-----------------------"
  )

  double contact;
  setupSimulation();
  resetWorld();
  contact = simulationLoop();

  RAIINFO(
      std::endl << "-----------------------" << std::endl
                << "Contacts : " << contact << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;
}