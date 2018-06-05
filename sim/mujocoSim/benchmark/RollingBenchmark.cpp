//
// Created by kangd on 21.02.18.
//

#include <MjcSim.hpp>

#include "MjcBenchmark.hpp"
#include "RollingBenchmark.hpp"

mujoco_sim::MjcSim *sim;
po::options_description desc;

void setupSimulation() {
  if (benchmark::rolling::options.gui)
    sim = new mujoco_sim::MjcSim(800, 600, 0.5,
                                 benchmark::rolling::getMujocoXMLpath().c_str(),
                                 benchmark::mujoco::getKeypath().c_str(),
                                 benchmark::NO_BACKGROUND,
                                 benchmark::mujoco::options.solverOption,
                                 benchmark::mujoco::options.integratorOption);
  else
    sim = new mujoco_sim::MjcSim(benchmark::rolling::getMujocoXMLpath().c_str(),
                                 benchmark::mujoco::getKeypath().c_str(),
                                 benchmark::mujoco::options.solverOption,
                                 benchmark::mujoco::options.integratorOption);

  // timestep
  sim->setTimeStep(benchmark::rolling::options.dt);

  /// no erp for mujoco
  if(benchmark::rolling::options.erpYN)
  RAIFATAL("erp is not supported for mujoco")
}

void setupWorld() {
  // gravity
  sim->setGravity({0, 0, benchmark::rolling::params.g});

  /// Note. for mujoco (frictional coefficient A-B) = max(coeff of A, coeff of B)
  sim->getSingleBodyHandle(0)->setFrictionCoefficient(benchmark::rolling::params.mjcGroundMu);
  sim->getSingleBodyHandle(1)->setFrictionCoefficient(benchmark::rolling::params.mjcBoxMu);

  if(benchmark::rolling::options.gui)
    sim->getSingleBodyHandle(1).visual()[0]->setColor({benchmark::mujoco::color[0],
                                                       benchmark::mujoco::color[1],
                                                       benchmark::mujoco::color[2]});
  for(int i = 2; i < sim->getNumObject(); i++) {
    sim->getSingleBodyHandle(i)->setFrictionCoefficient(benchmark::rolling::params.mjcBallMu);

    if(benchmark::rolling::options.gui)
      sim->getSingleBodyHandle(i).visual()[0]->setColor({benchmark::mujoco::color[0],
                                                         benchmark::mujoco::color[1],
                                                         benchmark::mujoco::color[2]});
  }

  if(benchmark::rolling::options.gui) {
    sim->setLightPosition((float)benchmark::rolling::params.lightPosition[0],
                          (float)benchmark::rolling::params.lightPosition[1],
                          (float)benchmark::rolling::params.lightPosition[2]);
    sim->cameraFollowObject(sim->getSingleBodyHandle(0), {30, 0, 15});
  }
}

void resetWorld() {
  sim->resetSimulation();
}

double simulationLoop(bool timer = true, bool error = true) {

  // force
  Eigen::Vector3d force;
  if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_Y)
    force = {0,
             benchmark::rolling::params.F,
             0};
  else if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY)
    force = {benchmark::rolling::params.F * 0.707106781186547,
             benchmark::rolling::params.F * 0.707106781186547,
             0};

  // resever error vector
  benchmark::rolling::data.setN(unsigned(benchmark::rolling::params.T / benchmark::rolling::options.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  // gui
  if(benchmark::rolling::options.saveVideo && benchmark::rolling::options.gui)
    sim->startRecordingVideo("/tmp", "mujoco-rolling");

  for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt); i++) {
    // gui
    if(benchmark::rolling::options.gui && !sim->visualizerLoop(benchmark::rolling::options.dt))
      break;

    // integrate step1
    sim->integrate1();

    // set force to box
    sim->getSingleBodyHandle(1)->setExternalForce(force);

    // data save
    if(error) {
      benchmark::rolling::data.boxVel.push_back(sim->getSingleBodyHandle(1)->getLinearVelocity());
      benchmark::rolling::data.boxPos.push_back(sim->getSingleBodyHandle(1)->getPosition());
      benchmark::rolling::data.ballVel.push_back(sim->getSingleBodyHandle(2)->getLinearVelocity());
      benchmark::rolling::data.ballPos.push_back(sim->getSingleBodyHandle(2)->getPosition());
    }

    // integrate step2
    sim->integrate2();
  }

  if(benchmark::rolling::options.saveVideo && benchmark::rolling::options.gui)
    sim->stopRecordingVideo();

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::rolling::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::rolling::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::rolling::getParamsFromYAML(benchmark::rolling::getYamlpath().c_str(),
                                        benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::rolling::options.gui << std::endl
                << "ERP      : " << benchmark::rolling::options.erpYN << std::endl
                << "Force    : " << benchmark::rolling::options.forceDirection << std::endl
                << "Timestep : " << benchmark::rolling::options.dt << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverName << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::rolling::data.computeError();

  // reset
  resetWorld();

  // trial2: get CPU time
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::rolling::options.csv)
    benchmark::rolling::printCSV(benchmark::rolling::getCSVpath(),
                                 benchmark::mujoco::options.simName,
                                 benchmark::mujoco::options.solverName,
                                 benchmark::mujoco::options.detectorName,
                                 benchmark::mujoco::options.integratorName,
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
