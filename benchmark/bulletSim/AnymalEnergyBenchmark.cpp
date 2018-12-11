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

  // checkerboard
  auto checkerboard = sim->addArticulatedSystem(
      benchmark::anymal::freedrop::getBulletPlanePath(),
      bullet_mb_sim::object::URDF
  );

  // anymal (no internal collision)
  auto anymal =
      sim->addArticulatedSystem(
          benchmark::anymal::freedrop::getBulletANYmalPath(),
          bullet_mb_sim::object::URDF,
          false
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
  benchmark::anymal::freedrop::params.F =
      benchmark::anymal::freedrop::params.M * (-benchmark::anymal::freedrop::params.g) * 2;

  if(benchmark::anymal::freedrop::options.gui)
    sim->cameraFollowObject(checkerboard, {25.0, 0.0, 7.0});
}

double simulationLoop(bool timer = true, bool error = true) {
  if(benchmark::anymal::freedrop::options.gui && benchmark::anymal::freedrop::options.saveVideo)
    sim->startRecordingVideo("/tmp", "bullet-anymal-energy");
  
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
      
      anymals[0]->setGeneralizedForce({0, 0, benchmark::anymal::freedrop::params.F,
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

      if(error) {
        if(t == 0)
          benchmark::anymal::freedrop::data.E0 = anymals[0]->getEnergy({0, 0, benchmark::anymal::freedrop::params.g});

        benchmark::anymal::freedrop::data.kineticE.push_back(
            anymals[0]->getKineticEnergy()
        );
        benchmark::anymal::freedrop::data.potentialE.push_back(
            anymals[0]->getPotentialEnergy({0, 0, benchmark::anymal::freedrop::params.g})
        );
      }
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
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::anymal::freedrop::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::freedrop::getParamsFromYAML(benchmark::anymal::freedrop::getYamlpath().c_str(),
                                                 benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator  : " << benchmark::bulletmultibody::options.simName << std::endl
                << "GUI        : " << benchmark::anymal::freedrop::options.gui << std::endl
                << "Solver     : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "Integrator : " << benchmark::bulletmultibody::options.integratorName << std::endl
                << "Timestep   : " << benchmark::anymal::freedrop::options.dt << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::anymal::freedrop::data.computeError();

  // reset
  anymals.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  int profileID = -1;
  if(benchmark::bulletmultibody::options.profiling)
    profileID= sim->startBulletProfiling("/tmp/btmb-anymal-energy-benchmark");
  double time = simulationLoop(true, false);
  if(benchmark::bulletmultibody::options.profiling)
    sim->stopBulletProfiling(profileID);


  if(benchmark::anymal::freedrop::options.csv)
    benchmark::anymal::freedrop::printCSV(benchmark::anymal::freedrop::getCSVpath(),
                                          benchmark::bulletmultibody::options.simName,
                                          benchmark::bulletmultibody::options.solverName,
                                          benchmark::bulletmultibody::options.detectorName,
                                          benchmark::bulletmultibody::options.integratorName,
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