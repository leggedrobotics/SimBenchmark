//
// Created by kangd on 14.05.18.
//

#include <raiSim/World_RG.hpp>

#include "AnymalEnergyBenchmark.hpp"

rai_sim::World_RG *sim;
rai_sim::ArticulatedSystemHandle anymal;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::freedrop::options.gui)
    sim = new rai_sim::World_RG(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG();

  // set erp 0
  sim->setERP(0);
}

void resetWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, 1, -1, rai_sim::GRID);

  // anymal (internal collision disabled)
  anymal = sim->addArticulatedSystem(
      benchmark::anymal::freedrop::getURDFpath(), 1, 0
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

  // gravity
  sim->setGravity({0, 0, benchmark::anymal::freedrop::params.g});

  // mass and force update
  benchmark::anymal::freedrop::params.M =
      std::accumulate( anymal->getMass().begin(), anymal->getMass().end(), 0.0);

  benchmark::anymal::freedrop::params.F =
      benchmark::anymal::freedrop::params.M * (-benchmark::anymal::freedrop::params.g) * 2;

  if(benchmark::anymal::freedrop::options.gui) {
    sim->cameraFollowObject(checkerboard, {25.0, 0.0, 7.0});

    // color
    for(int i = 0; i < anymal.visual().size(); i++) {
      anymal.visual()[i]->setColor({0.5373, 0.6471, 0.3059});
    }
  }
}

double computeEnergy() {
  return anymal->getEnergy({0, 0, benchmark::anymal::freedrop::params.g});
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
  if(benchmark::anymal::freedrop::options.gui) {
    // gui
    if(benchmark::anymal::freedrop::options.saveVideo)
      sim->startRecordingVideo("/tmp", "rai-anymal-freedrop");

    // step1: applying force
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor); t++) {

      sim->integrate1(benchmark::anymal::freedrop::options.dt);
      anymal->setGeneralizedForce({
                                      0, 0, benchmark::anymal::freedrop::params.F,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0});
      sim->integrate2(benchmark::anymal::freedrop::options.dt);
    }

    // step2: freedrop
    double E0 = 0;
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt) &&
        sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor); t++) {

      sim->integrate1(benchmark::anymal::freedrop::options.dt);
      if(t == 0)
        E0 = computeEnergy();
      anymal->setGeneralizedForce({
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0});
      benchmark::anymal::freedrop::data.errorList.push_back(computeEnergyError(E0));
      benchmark::anymal::freedrop::data.EList.push_back(computeEnergy());
      sim->integrate2(benchmark::anymal::freedrop::options.dt);
    }

    if(benchmark::anymal::freedrop::options.saveVideo)
      sim->stopRecordingVideo();

  } else {
    // step1: applying force
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {

      sim->integrate1(benchmark::anymal::freedrop::options.dt);
      anymal->setGeneralizedForce({
                                      0, 0, benchmark::anymal::freedrop::params.F,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0});
      sim->integrate2(benchmark::anymal::freedrop::options.dt);
    }

    // step2: freedrop
    double E0 = 0;
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {

      sim->integrate1(benchmark::anymal::freedrop::options.dt);
      if(t == 0)
        E0 = computeEnergy();
      anymal->setGeneralizedForce({
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0,
                                      0, 0, 0});
      benchmark::anymal::freedrop::data.errorList.push_back(computeEnergyError(E0));
      benchmark::anymal::freedrop::data.EList.push_back(computeEnergy());
      sim->integrate2(benchmark::anymal::freedrop::options.dt);
    }
  }

  double time = watch.measure();
  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(benchmark::anymal::freedrop::getCSVpath(),
                                          "RAI",
                                          "RAI",
                                          "RAI",
                                          "RAI",
                                          time);
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::freedrop::addDescToOption(desc);
  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::freedrop::getParamsFromYAML(benchmark::anymal::freedrop::getYamlpath().c_str(),
                                                 benchmark::RAI);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::anymal::freedrop::options.gui << std::endl
                << "Timestep : " << benchmark::anymal::freedrop::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  resetWorld();

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