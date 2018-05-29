//
// Created by kangd on 14.05.18.
//

#include <BtMbSim.hpp>

#include "AnymalEnergyBenchmark.hpp"
#include "BtMbBenchmark.hpp"

bullet_mb_sim::BtMbSim *sim;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> anymals;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::freedrop::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // set erp 0
  sim->setERP(0, 0, 0);
  sim->setTimeStep(benchmark::anymal::freedrop::options.dt);
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

  // anymal (no internal collision)
  auto anymal =
      sim->addArticulatedSystem(benchmark::anymal::freedrop::getURDFpath(), bullet_mb_sim::object::URDF, false);
  anymal->setGeneralizedCoordinate({0,
                                    0,
                                    benchmark::anymal::freedrop::params.H,
                                    1.0, 0.0, 0.0, 0.0,
                                    0.03, 0.4, -0.8,
                                    -0.03, 0.4, -0.8,
                                    0.03, -0.4, 0.8,
                                    -0.03, -0.4, 0.8});
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymals.push_back(anymal);

  // gravity
  sim->setGravity({0, 0, benchmark::anymal::freedrop::params.g});

  // mass
  benchmark::anymal::freedrop::params.M = anymal->getTotalMass();
  benchmark::anymal::freedrop::params.F =
      benchmark::anymal::freedrop::params.M * (-benchmark::anymal::freedrop::params.g) * 2;

  if(benchmark::anymal::freedrop::options.gui)
    sim->cameraFollowObject(checkerboard, {25.0, 0.0, 7.0});
}

double computeEnergy() {
  double energy = 0;
  for(int i = 0; i < anymals.size(); i++) {
    energy += anymals[i]->getEnergy({0, 0, benchmark::anymal::freedrop::params.g});
  }
  return energy;
}

double computeKineticEnergy() {
  double energy = 0;
  for(int i = 0; i < anymals.size(); i++) {
    energy += anymals[i]->getKineticEnergy();
  }
  return energy;
}

double computePotentialEnergy() {
  double energy = 0;
  for(int i = 0; i < anymals.size(); i++) {
    energy += anymals[i]->getPotentialEnergy({0, 0, benchmark::anymal::freedrop::params.g});
  }
  return energy;
}

double computeEnergyError(double E0) {
  // compute linear momentum
  return pow(computeEnergy() - E0, 2);
}

double simulationLoop() {

  // error list
  benchmark::anymal::freedrop::data.errorList.reserve(
      unsigned(benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt));
  benchmark::anymal::freedrop::data.EList.reserve(
      unsigned(benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt));

  StopWatch watch;
  watch.start();

//  anymals[0]->setGeneralizedCoordinate({
//                                           0, 0, 10,
//                                           1, 0, 0, 0,
//                                           0, 0, 0,
//                                           0, 0, 0,
//                                           0, 0, 0,
//                                           0, 0, 0
//                                       });
  if(benchmark::anymal::freedrop::options.gui) {
    // gui
    // step1: applying force
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor); t++) {

      anymals[0]->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0});
      sim->integrate();
    }

    // init E
    double E0 = computeEnergy();

    // step2: freedrop
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor); t++) {

      anymals[0]->setGeneralizedForce({0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0});
      benchmark::anymal::freedrop::data.errorList.push_back(computeEnergyError(E0));
      benchmark::anymal::freedrop::data.EList.push_back(computeEnergy());
//      benchmark::anymal::freedrop::data.kEList.push_back(computeKineticEnergy());
//      benchmark::anymal::freedrop::data.pEList.push_back(computePotentialEnergy());
      sim->integrate();
    }
  } else {
    // step1: applying force
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {

      anymals[0]->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0});
      sim->integrate();
    }

    // init E
    double E0 = computeEnergy();

    // step2: freedrop
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {

      anymals[0]->setGeneralizedForce({0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0,
                                       0, 0, 0});
      benchmark::anymal::freedrop::data.errorList.push_back(computeEnergyError(E0));
      benchmark::anymal::freedrop::data.EList.push_back(computeEnergy());
//      benchmark::anymal::freedrop::data.kEList.push_back(computeKineticEnergy());
//      benchmark::anymal::freedrop::data.pEList.push_back(computePotentialEnergy());
      sim->integrate();
    }
  }

  double time = watch.measure();
  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(benchmark::anymal::freedrop::getCSVpath(),
                                          benchmark::bulletmultibody::options.simName,
                                          benchmark::bulletmultibody::options.solverName,
                                          benchmark::bulletmultibody::options.detectorName,
                                          benchmark::bulletmultibody::options.integratorName,
                                          time);
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::freedrop::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::freedrop::getParamsFromYAML(benchmark::anymal::freedrop::getYamlpath().c_str(),
                                                 benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: " << benchmark::bulletmultibody::options.simName << std::endl
                << "GUI      : " << benchmark::anymal::freedrop::options.gui << std::endl
                << "Solver   : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "Timestep : " << benchmark::anymal::freedrop::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();

  RAIINFO(
      std::endl << "Timer    : " << simulationLoop() << std::endl
                << "Mean Error: " << benchmark::anymal::freedrop::computeMeanError() << std::endl
                << "Contacts  : " << sim->getWorldNumContacts() << std::endl
                << "======================="
  )

  // show plot
  if(benchmark::anymal::freedrop::options.plot) {
    benchmark::anymal::freedrop::showPlot();
  }

  delete sim;
  return 0;
}