//
// Created by kangd on 26.04.18.
//

#include <DartSim.hpp>

#include "AtlasContactBenchmark.hpp"
#include "DartBenchmark.hpp"
#include "raiCommon/utils/StopWatch.hpp"

dart_sim::DartSim *sim;
std::vector<dart_sim::ArticulatedSystemHandle> robots;
po::options_description desc;

void setupSimulation() {
  if(benchmark::atlas::options.gui)
    sim = new dart_sim::DartSim(800, 600, 0.5,
                                benchmark::NO_BACKGROUND,
                                benchmark::dart::options.solverOption,
                                benchmark::dart::options.detectorOption);
  else
    sim = new dart_sim::DartSim(benchmark::dart::options.solverOption,
                                benchmark::dart::options.detectorOption);

  // time step
  sim->setTimeStep(benchmark::atlas::params.dt);
  sim->setMaxContacts(5000);
}

void resetWorld() {
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0.8);

  for(int i = 0; i < benchmark::atlas::options.numRow; i++) {
    for(int j = 0; j < benchmark::atlas::options.numRow; j++) {
      auto robot = sim->addArticulatedSystem(
          benchmark::atlas::getURDFpath()
      );
//      atlas->setColor({1, 0, 0, 1});

      Eigen::VectorXd gc(robot->getStateDimension());
      Eigen::VectorXd gv(robot->getDOF());
      Eigen::VectorXd tau(robot->getDOF());
      gc.setZero();
      gc.segment<7>(0) << 2.5 * i, 2.5 * j, benchmark::atlas::params.H,
          benchmark::atlas::params.baseQuat[0],
          benchmark::atlas::params.baseQuat[1],
          benchmark::atlas::params.baseQuat[2],
          benchmark::atlas::params.baseQuat[3];

      robot->setGeneralizedCoordinate(gc);
      robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
      robots.push_back(robot);
    }
  }

  sim->setGravity({0, 0, benchmark::atlas::params.g});

  if(benchmark::atlas::options.gui)
    sim->cameraFollowObject(checkerboard, {1.0, 1.0, 1.0});
}

double simulationLoop(bool timer = true, bool cntNumContact = true) {
  // resever error vector
  benchmark::atlas::data.setN(unsigned(benchmark::atlas::params.T / benchmark::atlas::params.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  Eigen::VectorXd gc(robots[0]->getStateDimension());
  Eigen::VectorXd gv(robots[0]->getDOF());
  Eigen::VectorXd tau(robots[0]->getDOF());

  if(benchmark::atlas::options.gui) {
    // gui
    while(sim->visualizerLoop(benchmark::atlas::params.dt)) {
      for(int i = 0; i < robots.size(); i++) {
        gc = robots[i]->getGeneralizedCoordinate();
        gv = robots[i]->getGeneralizedVelocity();
        tau.tail(30) =
            -benchmark::atlas::params.kp.tail(30).cwiseProduct(gc.tail(30))
                - benchmark::atlas::params.kd.cwiseProduct(gv).tail(30);
        robots[i]->setGeneralizedForce(tau);
      }
      sim->integrate();
      if(cntNumContact) benchmark::atlas::data.numContactList.push_back(sim->getWorldNumContacts());
    }
  } else {
    // no gui
    for(int t = 0; t < (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt); t++) {
      for(int i = 0; i < robots.size(); i++) {
        gc = robots[i]->getGeneralizedCoordinate();
        gv = robots[i]->getGeneralizedVelocity();
        tau.tail(30) =
            -benchmark::atlas::params.kp.tail(30).cwiseProduct(gc.tail(30))
                - benchmark::atlas::params.kd.cwiseProduct(gv).tail(30);
        robots[i]->setGeneralizedForce(tau);
      }
      sim->integrate();
      if(cntNumContact) benchmark::atlas::data.numContactList.push_back(sim->getWorldNumContacts());
    }
  }
  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::atlas::addDescToOption(desc);
  benchmark::dart::addDescToOption(desc);

  benchmark::atlas::getOptionsFromArg(argc, argv, desc);
  benchmark::dart::getOptionsFromArg(argc, argv, desc);

  benchmark::atlas::getParamsFromYAML(benchmark::atlas::getYamlpath().c_str(),
                                      benchmark::DART);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: DART" << std::endl
                << "GUI      : " << benchmark::atlas::options.gui << std::endl
                << "Row      : " << benchmark::atlas::options.numRow << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  resetWorld();
  simulationLoop(false, true);
  double avgNumContacts = benchmark::atlas::data.computeAvgNumContact();

  // reset
  robots.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  resetWorld();
  double time = simulationLoop(true, false);

  // print to screen
  std::cout<<"time taken for "
           << (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt)
           << " steps "<< time <<"s \n";

  if(benchmark::atlas::options.csv)
    benchmark::atlas::printCSV(benchmark::atlas::getCSVpath(),
                               benchmark::dart::options.simName,
                               benchmark::dart::options.solverName,
                               benchmark::dart::options.detectorName,
                               benchmark::dart::options.integratorName,
                               benchmark::atlas::options.numRow,
                               avgNumContacts,
                               time);

  delete sim;
  return 0;
}