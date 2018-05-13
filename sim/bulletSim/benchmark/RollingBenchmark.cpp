//
// Created by kangd on 15.02.18.
//

#include <BtWorld_RG.hpp>

#include "RollingBenchmark.hpp"
#include "BtBenchmark.hpp"

std::vector<double> errors;
bullet_sim::BtWorld_RG *sim;
std::vector<benchmark::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::rolling::options.gui)
    sim = new bullet_sim::BtWorld_RG(800, 600, 0.5,
                                     benchmark::NO_BACKGROUND,
                                     benchmark::bullet::options.solverOption);
  else
    sim = new bullet_sim::BtWorld_RG(benchmark::bullet::options.solverOption);

  // erp
  if(benchmark::rolling::options.erpYN)
    sim->setERP(benchmark::rolling::params.erp,
                benchmark::rolling::params.erp2,
                benchmark::rolling::params.erpFriction);
  else
    sim->setERP(0, 0, 0);

  // set up logger and timer
  if(benchmark::rolling::options.log)
    benchmark::rolling::loggerSetup(
        benchmark::rolling::getLogDirpath(benchmark::rolling::options.erpYN,
                                          benchmark::rolling::options.forceDirection,
                                          benchmark::bullet::options.simName,
                                          benchmark::bullet::options.solverName,
                                          benchmark::rolling::options.dt), "var"
    );
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(benchmark::rolling::params.btGroundMu);

  auto box = sim->addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5 - benchmark::rolling::params.initPenetration);
  box->setFrictionCoefficient(benchmark::rolling::params.btBoxMu);
  objList.push_back(box);

  for(int i = 0; i < benchmark::rolling::params.n; i++) {
    for(int j = 0; j < benchmark::rolling::params.n; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0,
                        j * 2.0 - 4.0,
                        1.5 - 3 * benchmark::rolling::params.initPenetration);
      ball->setFrictionCoefficient(benchmark::rolling::params.btBallMu);
      objList.push_back(ball);
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

Eigen::Vector3d computeAnalyticalSol(double t, bool isBall) {
  const double g = -benchmark::rolling::params.g;
  const double m = benchmark::rolling::params.m;
  const double M = benchmark::rolling::params.M;
  const double F = benchmark::rolling::params.F;
  const int n = benchmark::rolling::params.n * benchmark::rolling::params.n;
  const double mu1 = benchmark::rolling::params.raiGroundMu * benchmark::rolling::params.raiBoxMu;
  const double mu2 = benchmark::rolling::params.raiBoxMu * benchmark::rolling::params.raiBallMu;

  const double simTime = benchmark::rolling::params.T;
  const double dt = benchmark::rolling::options.dt;

  double f1 = mu1 * (M + n * m)  * g;
  double f2 = 1 / M * (150 - f1) / (3.5 / m + 25 / M);
  double a1 = (F - f1 - n * f2) / M;
  double a2 = f2 / m;

  double v = 0;
  if (isBall)
    v = a2 * t;
  else
    v = a1 * t;

  if (benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY)
    return {v * 0.707106781186547, v * 0.707106781186547, 0};
  else
    return {0, v, 0};
}

void simulationLoop() {

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
  errors.reserve(unsigned(benchmark::rolling::params.T / benchmark::rolling::options.dt));

  if(benchmark::rolling::options.gui) {
    // gui
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

      Eigen::Vector3d ballVec = computeAnalyticalSol(benchmark::rolling::options.dt * i, true);
      Eigen::Vector3d boxVec = computeAnalyticalSol(benchmark::rolling::options.dt * i, false);

      double error = 0;
      error += pow((boxVec - objList[0]->getLinearVelocity()).norm(), 2);
      error += pow((ballVec - objList[1]->getLinearVelocity()).norm(), 2);
      errors.push_back(error);

      sim->integrate(benchmark::rolling::options.dt);
    }
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

      Eigen::Vector3d ballVec = computeAnalyticalSol(benchmark::rolling::options.dt * i, true);
      Eigen::Vector3d boxVec = computeAnalyticalSol(benchmark::rolling::options.dt * i, false);

      double error = 0;
      error += pow((boxVec - objList[0]->getLinearVelocity()).norm(), 2);
      error += pow((ballVec - objList[1]->getLinearVelocity()).norm(), 2);
      errors.push_back(error);

      sim->integrate(benchmark::rolling::options.dt);
    }

    if(benchmark::rolling::options.log)
      ru::timer->stopTimer("rolling");
  }
}

int main(int argc, const char* argv[]) {

  benchmark::rolling::addDescToOption(desc);
  benchmark::bullet::addDescToOption(desc);

  benchmark::rolling::getOptionsFromArg(argc, argv, desc);
  benchmark::bullet::getOptionsFromArg(argc, argv, desc);

  benchmark::rolling::getParamsFromYAML(benchmark::rolling::getYamlpath().c_str(),
                                        benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: BULLET" << std::endl
                << "GUI      : " << benchmark::rolling::options.gui << std::endl
                << "ERP      : " << benchmark::rolling::options.erpYN << std::endl
                << "Force    : " << benchmark::rolling::options.forceDirection << std::endl
                << "Timestep : " << benchmark::rolling::options.dt << std::endl
                << "Solver   : " << benchmark::bullet::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  // time log
  if(benchmark::rolling::options.log)
    ru::timer->dumpToStdOuput();

  RAIINFO("mean error = " << std::accumulate( errors.begin(), errors.end(), 0.0) / errors.size();)

  delete sim;
  return 0;
}