//
// Created by kangd on 15.02.18.
//

#include <BtMbSim.hpp>

#include "RollingBenchmark.hpp"
#include "BtMbBenchmark.hpp"

bullet_mb_sim::BtMbSim *sim;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::rolling::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // erp
  if(benchmark::rolling::options.erpYN)
    sim->setERP(benchmark::rolling::params.erp,
                benchmark::rolling::params.erp2,
                benchmark::rolling::params.erpFriction);
  else
    sim->setERP(0, 0, 0);

  // time step
  sim->setTimeStep(benchmark::rolling::options.dt);

  // set up logger and timer
  if(benchmark::rolling::options.log)
    benchmark::rolling::loggerSetup(
        benchmark::rolling::getLogDirpath(benchmark::rolling::options.erpYN,
                                          benchmark::rolling::options.forceDirection,
                                          benchmark::bulletmultibody::options.simName,
                                          benchmark::bulletmultibody::options.solverName,
                                          benchmark::bulletmultibody::options.detectorName,
                                          benchmark::bulletmultibody::options.integratorName,
                                          benchmark::rolling::options.dt), "var"
    );
}

void setupWorld() {

  // add objects
  auto checkerboard = sim->addArticulatedSystem(benchmark::rolling::getBulletPlanepath(), bullet_mb_sim::object::URDF);
  checkerboard->setFrictionCoefficient(-1, benchmark::rolling::params.btGroundMu);

  auto box = sim->addArticulatedSystem(benchmark::rolling::getBulletBoxpath(), bullet_mb_sim::object::URDF);
  box->setGeneralizedCoordinate({0, 0, 0.5 - benchmark::rolling::params.initPenetration,
                                 1, 0, 0, 0});
  box->setFrictionCoefficient(-1, benchmark::rolling::params.btBoxMu);
  objList.push_back(box);

  for(int i = 0; i < benchmark::rolling::params.n; i++) {
    for(int j = 0; j < benchmark::rolling::params.n; j++) {
      auto ball = sim->addArticulatedSystem(benchmark::rolling::getBulletBallpath(), bullet_mb_sim::object::URDF);
      ball->setGeneralizedCoordinate({i * 2.0 - 4.0, j * 2.0 - 4.0,
                                      1.5 - 3 * benchmark::rolling::params.initPenetration,
                                      1, 0, 0, 0});
      ball->setFrictionCoefficient(-1, benchmark::rolling::params.btBallMu);
      objList.push_back(ball);
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::rolling::params.g});

  if(benchmark::rolling::options.gui) {
    sim->setLightPosition((float)benchmark::rolling::params.lightPosition[0],
                          (float)benchmark::rolling::params.lightPosition[1],
                          (float)benchmark::rolling::params.lightPosition[2]);
//    sim->cameraFollowObject(checkerboard, {30, 0, 15});
  }
}

double simulationLoop() {

  // force
  Eigen::VectorXd force(6);
  if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_Y)
    force << 0, benchmark::rolling::params.F, 0, 0, 0, 0;
  else if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY)
    force << benchmark::rolling::params.F * 0.707106781186547,
             benchmark::rolling::params.F * 0.707106781186547,
             0, 0, 0, 0;

  // resever error vector
  benchmark::rolling::errors.reserve(unsigned(benchmark::rolling::params.T / benchmark::rolling::options.dt));

  StopWatch watch;
  watch.start();
  if(benchmark::rolling::options.gui) {
    // gui
    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt) &&
        sim->visualizerLoop(benchmark::rolling::options.dt); i++) {

      // set force to box
      objList[0]->setGeneralizedForce(force);

      // log
      if(benchmark::rolling::options.log) {
        Eigen::Vector3d velbox = objList[0]->getGeneralizedVelocity().head(3);
        Eigen::Vector3d velball = objList[1]->getGeneralizedVelocity().head(3);
        Eigen::Vector3d posbox = objList[0]->getGeneralizedCoordinate().head(3);
        Eigen::Vector3d posball = objList[1]->getGeneralizedCoordinate().head(3);
        ru::logger->appendData("velbox", velbox.data());
        ru::logger->appendData("velball", velball.data());
        ru::logger->appendData("posbox", posbox.data());
        ru::logger->appendData("posball", posball.data());
      }

      Eigen::Vector3d ballVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, true);
      Eigen::Vector3d boxVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, false);

      double error = 0;
      error += pow((boxVec - objList[0]->getGeneralizedVelocity().head(3)).norm(), 2);
      error += pow((ballVec - objList[1]->getGeneralizedVelocity().head(3)).norm(), 2);
      benchmark::rolling::errors.push_back(error);

      sim->integrate();
    }
  }
  else {
    // no gui
    if(benchmark::rolling::options.log)
      ru::timer->startTimer("rolling");

    for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt); i++) {

      // set force to box
      objList[0]->setGeneralizedForce(force);

      // log
      if(benchmark::rolling::options.log) {
        Eigen::Vector3d velbox = objList[0]->getGeneralizedVelocity().head(3);
        Eigen::Vector3d velball = objList[1]->getGeneralizedVelocity().head(3);
        Eigen::Vector3d posbox = objList[0]->getGeneralizedCoordinate().head(3);
        Eigen::Vector3d posball = objList[1]->getGeneralizedCoordinate().head(3);
        ru::logger->appendData("velbox", velbox.data());
        ru::logger->appendData("velball", velball.data());
        ru::logger->appendData("posbox", posbox.data());
        ru::logger->appendData("posball", posball.data());
      }

      Eigen::Vector3d ballVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, true);
      Eigen::Vector3d boxVec = benchmark::rolling::computeAnalyticalSol(benchmark::rolling::options.dt * i, false);

      double error = 0;
      error += pow((boxVec - objList[0]->getGeneralizedVelocity().head(3)).norm(), 2);
      error += pow((ballVec - objList[1]->getGeneralizedVelocity().head(3)).norm(), 2);
      benchmark::rolling::errors.push_back(error);

      sim->integrate();
    }

    if(benchmark::rolling::options.log)
      ru::timer->stopTimer("rolling");
  }

  return watch.measure();
}

int main(int argc, const char* argv[]) {

  benchmark::rolling::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::rolling::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::rolling::getParamsFromYAML(benchmark::rolling::getYamlpath().c_str(),
                                        benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: BULLET" << std::endl
                << "GUI      : " << benchmark::rolling::options.gui << std::endl
                << "ERP      : " << benchmark::rolling::options.erpYN << std::endl
                << "Force    : " << benchmark::rolling::options.forceDirection << std::endl
                << "Timestep : " << benchmark::rolling::options.dt << std::endl
                << "Solver   : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  double time = simulationLoop();

  if(benchmark::rolling::options.csv)
    benchmark::rolling::printCSV(benchmark::rolling::getCSVpath(),
                                 benchmark::bulletmultibody::options.simName,
                                 benchmark::bulletmultibody::options.solverName,
                                 benchmark::bulletmultibody::options.detectorName,
                                 benchmark::bulletmultibody::options.integratorName,
                                 time);

  RAIINFO(
      std::endl << "time       : " << time << std::endl
                << "mean error : " << benchmark::rolling::computeMeanError() << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;
}