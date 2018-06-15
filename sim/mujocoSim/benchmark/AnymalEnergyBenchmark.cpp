//
// Created by kangd on 14.05.18.
//

#include <MjcSim.hpp>

#include "AnymalEnergyBenchmark.hpp"
#include "MjcBenchmark.hpp"

mujoco_sim::MjcSim *sim;
po::options_description desc;

void setupSimulation() {
  if(benchmark::anymal::freedrop::options.gui)
    sim = new mujoco_sim::MjcSim(800, 600, 0.5,
                                 benchmark::anymal::freedrop::getMujocoURDFpath().c_str(),
                                 benchmark::mujoco::getKeypath().c_str(),
                                 benchmark::NO_BACKGROUND,
                                 benchmark::mujoco::options.solverOption,
                                 benchmark::mujoco::options.integratorOption);
  else
    sim = new mujoco_sim::MjcSim(benchmark::anymal::freedrop::getMujocoURDFpath().c_str(),
                                 benchmark::mujoco::getKeypath().c_str(),
                                 benchmark::mujoco::options.solverOption,
                                 benchmark::mujoco::options.integratorOption);

  // set time step
  sim->setTimeStep(benchmark::anymal::freedrop::options.dt);
}

void setupWorld() {

  sim->setGeneralizedCoordinate({0, 0, benchmark::anymal::freedrop::params.H,
                                 1.0, 0.0, 0.0, 0.0,
                                 0.03, 0.4, -0.8,
                                 0.03, -0.4, +0.8,
                                 -0.03, 0.4, -0.8,
                                 -0.03, -0.4, 0.8});
  sim->setGeneralizedForce(Eigen::VectorXd::Zero(sim->getDOF()));

  // gravity
  sim->setGravity({0, 0, benchmark::anymal::freedrop::params.g});

  // mass
  benchmark::anymal::freedrop::params.M = sim->getTotalMass();
  benchmark::anymal::freedrop::params.F =
      benchmark::anymal::freedrop::params.M * (-benchmark::anymal::freedrop::params.g) * 2;

  // no collision
  sim->setWorldContactFlag(false);

  if(benchmark::anymal::freedrop::options.gui)
    sim->cameraFollowObject(
        sim->getSingleBodyHandle(sim->getNumObject()-1), {25.0, 0.0, 7.0});
}


void resetWorld() {
  sim->resetSimulation();
}

double simulationLoop(bool timer = true, bool error = true) {
  if(benchmark::anymal::freedrop::options.gui && benchmark::anymal::freedrop::options.saveVideo)
    sim->startRecordingVideo("/tmp", "mujoco-anymal-energy");

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
          !sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor))
        break;

      sim->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
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
          !sim->visualizerLoop(benchmark::anymal::freedrop::options.dt, benchmark::anymal::freedrop::options.guiRealtimeFactor))
        break;

      // integrate step1
      sim->forwardKinematics();
      sim->setGeneralizedForce({0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 0});

      if(error) {
        if(t==0)
          benchmark::anymal::freedrop::data.E0 = sim->getEnergy({0, 0, benchmark::anymal::freedrop::params.g});

        benchmark::anymal::freedrop::data.kineticE.push_back(
            sim->getKineticEnergy()
        );
        benchmark::anymal::freedrop::data.potentialE.push_back(
            sim->getPotentialEnergy({0, 0, benchmark::anymal::freedrop::params.g})
        );
      }

      // step 2
      sim->integrate();
    }
  }

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::freedrop::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::freedrop::getParamsFromYAML(benchmark::anymal::freedrop::getYamlpath().c_str(),
                                                 benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator  : " << benchmark::mujoco::options.simName << std::endl
                << "GUI        : " << benchmark::anymal::freedrop::options.gui << std::endl
                << "Solver     : " << benchmark::mujoco::options.solverName << std::endl
                << "Integrator : " << benchmark::mujoco::options.integratorName << std::endl
                << "Timestep   : " << benchmark::anymal::freedrop::options.dt << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::anymal::freedrop::data.computeError();

  // reset
  resetWorld();

  // trial2: get CPU time
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(benchmark::anymal::freedrop::getCSVpath(),
                                          benchmark::mujoco::options.simName,
                                          benchmark::mujoco::options.solverName,
                                          benchmark::mujoco::options.detectorName,
                                          benchmark::mujoco::options.integratorName,
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