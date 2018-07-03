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

  // time step
  sim->setTimeStep(benchmark::anymal::freedrop::options.dt);
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(2, 100, 100, 0.1, -1, rai_sim::GRID);

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

double simulationLoop(bool timer = true, bool error = true) {
  if(benchmark::anymal::freedrop::options.gui && benchmark::anymal::freedrop::options.saveVideo)
    sim->startRecordingVideo("/tmp", "rai-anymal-energy");

  // resever error vector
  if(error)
    benchmark::anymal::freedrop::data.setN(
        unsigned(benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt)
    );

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  {
    // step1: applying force
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T1 / benchmark::anymal::freedrop::options.dt); t++) {
      if(benchmark::anymal::freedrop::options.gui &&
          !sim->visualizerLoop(benchmark::anymal::freedrop::options.guiRealtimeFactor))
        break;

      anymal->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0});
      sim->integrate();
    }
  }

  {
    // step2: freedrop
    for (int t = 0; t < (int) (benchmark::anymal::freedrop::params.T2 / benchmark::anymal::freedrop::options.dt); t++) {
      if(benchmark::anymal::freedrop::options.gui &&
          !sim->visualizerLoop(benchmark::anymal::freedrop::options.guiRealtimeFactor))
        break;

      sim->integrate1();

      anymal->setGeneralizedForce({0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0});

      if(error) {
        if(t==0)
          benchmark::anymal::freedrop::data.E0 = anymal->getEnergy({0, 0, benchmark::anymal::freedrop::params.g});

        benchmark::anymal::freedrop::data.kineticE.push_back(
            anymal->getKineticEnergy()
        );
        benchmark::anymal::freedrop::data.potentialE.push_back(
            anymal->getPotentialEnergy({0, 0, benchmark::anymal::freedrop::params.g})
        );
      }
      sim->integrate2();
    }
  }

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::freedrop::addDescToOption(desc);
  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::freedrop::getParamsFromYAML(benchmark::anymal::freedrop::getYamlpath().c_str(),
                                                 benchmark::RAI);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator  : " << "RAI" << std::endl
                << "GUI        : " << benchmark::anymal::freedrop::options.gui << std::endl
                << "Solver     : " << "RAI" << std::endl
                << "Integrator : " << "RAI" << std::endl
                << "Timestep   : " << benchmark::anymal::freedrop::options.dt << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::anymal::freedrop::data.computeError();

  // reset
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(benchmark::anymal::freedrop::getCSVpath(),
                                          "RAI",
                                          "RAI",
                                          "RAI",
                                          "RAI",
                                          time,
                                          error);

  RAIINFO(
      std::endl << "CPU Timer : " << time << std::endl
                << "Mean Error: " << error << std::endl
                << "======================="
  )

  delete sim;
  return 0;
}