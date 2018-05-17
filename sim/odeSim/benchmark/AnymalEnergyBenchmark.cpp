//
// Created by kangd on 14.05.18.
//

#include <OdeWorld_RG.hpp>

#include "AnymalEnergyBenchmark.hpp"
#include "OdeBenchmark.hpp"

ode_sim::OdeWorld_RG *sim;
std::vector<ode_sim::ArticulatedSystemHandle> anymals;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::freedrop::options.gui)
    sim = new ode_sim::OdeWorld_RG(800, 600, 0.5,
                                   benchmark::NO_BACKGROUND,
                                   benchmark::ode::options.solverOption);
  else
    sim = new ode_sim::OdeWorld_RG(benchmark::ode::options.solverOption);

  // set erp 0
  sim->setERP(0, 0, 0);
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);

  // anymal
  auto anymal = sim->addArticulatedSystem(
      benchmark::anymal::freedrop::getURDFpath()
  );
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

  if(benchmark::anymal::freedrop::options.gui)
    sim->cameraFollowObject(checkerboard, {10.0, 0.0, 30.0});
}

double computeEnergy() {
  double energy = 0;
  for(int i = 0; i < anymals.size(); i++) {
    energy += anymals[i]->getEnergy({0, 0, benchmark::anymal::freedrop::params.g});
  }
  return energy;
}

double computeEnergyError(double E0) {
  // compute linear momentum
  return pow(computeEnergy() - E0, 2);
}


double simulationLoop() {

  // error list
  benchmark::anymal::freedrop::errorList.reserve(
      unsigned(benchmark::anymal::freedrop::params.T / benchmark::anymal::freedrop::options.dt));

  // E0
  double E0 = computeEnergy();

  StopWatch watch;
  watch.start();
  if(benchmark::anymal::freedrop::options.gui) {
    // gui
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T / benchmark::anymal::freedrop::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor); t++) {

      benchmark::anymal::freedrop::errorList.push_back(computeEnergyError(E0));
      sim->integrate(benchmark::anymal::freedrop::options.dt);
    }
  } else {
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T / benchmark::anymal::freedrop::options.dt); t++) {
      benchmark::anymal::freedrop::errorList.push_back(computeEnergyError(E0));
      sim->integrate(benchmark::anymal::freedrop::options.dt);
    }
  }

  double time = watch.measure();
  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(
        benchmark::anymal::freedrop::getCSVpath(),
        benchmark::ode::options.simName,
        benchmark::ode::options.solverName,
        time);
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::freedrop::addDescToOption(desc);
  benchmark::ode::addDescToOption(desc);

  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);
  benchmark::ode::getOptionsFromArg(argc, argv, desc);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: ODE" << std::endl
                << "GUI      : " << benchmark::anymal::freedrop::options.gui << std::endl
                << "Solver   : " << benchmark::ode::options.solverOption << std::endl
                << "Timestep : " << benchmark::anymal::freedrop::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();

  RAIINFO(
      std::endl << "Timer    : " << simulationLoop() << std::endl
                << "Mean Error: " << benchmark::anymal::freedrop::computeMeanError() << std::endl
                << "======================="
  )

  // show plot
  if(benchmark::anymal::freedrop::options.plot) {
    benchmark::anymal::freedrop::showPlot();
  }

  delete sim;
  return 0;
}