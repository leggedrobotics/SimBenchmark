//
// Created by kangd on 15.02.18.
//

#include "OdeWorld_RG.hpp"

#include "RollingBenchmark.hpp"
#include "OdeBenchmark.hpp"

ode_sim::OdeWorld_RG *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::rolling::options.gui)
    sim = new ode_sim::OdeWorld_RG(800, 600, 0.5,
                                   benchmark::NO_BACKGROUND,
                                   benchmark::ode::options.solverOption);
  else
    sim = new ode_sim::OdeWorld_RG(benchmark::ode::options.solverOption);

  // erp
  if(benchmark::rolling::options.erpYN)
    sim->setERP(benchmark::rolling::params.erp, 0, 0);
  else
    sim->setERP(0, 0, 0);

  // set up logger and timer
  if(benchmark::rolling::options.log)
    benchmark::rolling::loggerSetup(
        benchmark::rolling::getLogDirpath(benchmark::rolling::options.erpYN,
                                          benchmark::rolling::options.forceDirection,
                                          benchmark::ode::options.simName,
                                          benchmark::ode::options.solverName,
                                          benchmark::rolling::options.dt), "var"
    );
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(benchmark::rolling::params.odeGroundMu);

  auto box = sim->addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5 - benchmark::rolling::params.initPenetration);
  box->setFrictionCoefficient(benchmark::rolling::params.odeBoxMu);
  objList.push_back(box);

  if(benchmark::rolling::options.gui)
    box.visual()[0]->setColor(
        {benchmark::ode::color[0],
         benchmark::ode::color[1],
         benchmark::ode::color[2]});

  for(int i = 0; i < benchmark::rolling::params.n; i++) {
    for(int j = 0; j < benchmark::rolling::params.n; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0,
                        j * 2.0 - 4.0,
                        1.5 - 3 * benchmark::rolling::params.initPenetration);
      ball->setFrictionCoefficient(benchmark::rolling::params.odeBallMu);
      objList.push_back(ball);

      if(benchmark::rolling::options.gui)
        ball.visual()[0]->setColor(
            {benchmark::ode::color[0],
             benchmark::ode::color[1],
             benchmark::ode::color[2]});
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::rolling::params.g});

  if(benchmark::rolling::options.gui) {
    sim->setLightPosition((float)benchmark::rolling::params.lightPosition[0],
                          (float)benchmark::rolling::params.lightPosition[1],
                          (float)benchmark::rolling::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {30, 0, 15});
  }
}

double simulationLoop() {

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
  benchmark::rolling::errors.reserve(unsigned(benchmark::rolling::params.T / benchmark::rolling::options.dt));

  StopWatch watch;
  watch.start();
  if(benchmark::rolling::options.gui) {
    // gui
    if(benchmark::rolling::options.saveVideo)
      sim->startRecordingVideo("/tmp", "ode-rolling");

    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt) &&
        sim->visualizerLoop(benchmark::rolling::options.dt); i++) {

      // set force to box
      objList[0]->setExternalForce(force);

      // log
      if(benchmark::rolling::options.log) {
        ru::logger->appendData("velbox", objList[0]->getLinearVelocity().data());
        ru::logger->appendData("velball", objList[1]->getLinearVelocity().data());
        ru::logger->appendData("posbox", objList[0]->getPosition().data());
        ru::logger->appendData("posball", objList[1]->getPosition().data());
      }

      Eigen::Vector3d ballVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, true);
      Eigen::Vector3d boxVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, false);

      double error = 0;
      error += pow((boxVec - objList[0]->getLinearVelocity()).norm(), 2);
      error += pow((ballVec - objList[1]->getLinearVelocity()).norm(), 2);
      benchmark::rolling::errors.push_back(error);

      sim->integrate(benchmark::rolling::options.dt);
    }

    if(benchmark::rolling::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    if(benchmark::rolling::options.log)
      ru::timer->startTimer("rolling");

    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt); i++) {

      // set force to box
      objList[0]->setExternalForce(force);

      // log
      if(benchmark::rolling::options.log) {
        ru::logger->appendData("velbox", objList[0]->getLinearVelocity().data());
        ru::logger->appendData("velball", objList[1]->getLinearVelocity().data());
        ru::logger->appendData("posbox", objList[0]->getPosition().data());
        ru::logger->appendData("posball", objList[1]->getPosition().data());
      }

      Eigen::Vector3d ballVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, true);
      Eigen::Vector3d boxVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, false);

      double error = 0;
      error += pow((boxVec - objList[0]->getLinearVelocity()).norm(), 2);
      error += pow((ballVec - objList[1]->getLinearVelocity()).norm(), 2);
      benchmark::rolling::errors.push_back(error);

      sim->integrate(benchmark::rolling::options.dt);
    }

    if(benchmark::rolling::options.log)
      ru::timer->stopTimer("rolling");
  }

  return watch.measure();
}

int main(int argc, const char* argv[]) {

  benchmark::rolling::addDescToOption(desc);
  benchmark::ode::addDescToOption(desc);

  benchmark::rolling::getOptionsFromArg(argc, argv, desc);
  benchmark::ode::getOptionsFromArg(argc, argv, desc);

  benchmark::rolling::getParamsFromYAML(benchmark::rolling::getYamlpath().c_str(),
                                        benchmark::ODE);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: ODE" << std::endl
                << "GUI      : " << benchmark::rolling::options.gui << std::endl
                << "ERP      : " << benchmark::rolling::options.erpYN << std::endl
                << "Force    : " << benchmark::rolling::options.forceDirection << std::endl
                << "Timestep : " << benchmark::rolling::options.dt << std::endl
                << "Solver   : " << benchmark::ode::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  double time = simulationLoop();

  if(benchmark::rolling::options.csv)
    benchmark::rolling::printCSV(
        benchmark::rolling::getCSVpath(),
        benchmark::ode::options.simName,
        benchmark::ode::options.solverName,
        time);

  RAIINFO(
      std::endl << "time       : " << time << std::endl
                << "mean error : " << benchmark::rolling::computeMeanError() << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;
}