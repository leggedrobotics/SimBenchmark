//
// Created by kangd on 15.02.18.
//

#include <BtMbSim.hpp>

#include "RollingBenchmark.hpp"
#include "BtMbBenchmark.hpp"

// sim
bullet_mb_sim::BtMbSim *sim;

// objects
std::vector<bullet_mb_sim::ArticulatedSystemHandle> objList;

// options description
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

  // solver iteration
  sim->setSolverParameter(1e-30,
                          benchmark::rolling::options.numSolverIter);
}

void setupWorld() {
  // plane
  auto checkerboard = sim->addArticulatedSystem(
      benchmark::rolling::getBulletPlanePath(),
      bullet_mb_sim::object::URDF,
      false
  );
  checkerboard->setFrictionCoefficient(-1, benchmark::rolling::params.btGroundMu);

  // box
  auto box = sim->addArticulatedSystem(
      benchmark::rolling::getBulletBoxPath(),
      bullet_mb_sim::object::URDF,
      false
  );
  box->setGeneralizedCoordinate(
      {0, 0, 0.5 - benchmark::rolling::params.initPenetration,
       1, 0, 0, 0}
  );
  box->setFrictionCoefficient(-1, benchmark::rolling::params.btBoxMu);
  objList.push_back(box);
  
  // balls
  for(int i = 0; i < benchmark::rolling::params.n; i++) {
    for(int j = 0; j < benchmark::rolling::params.n; j++) {
      auto ball = sim->addArticulatedSystem(
          benchmark::rolling::getBulletBallPath(),
          bullet_mb_sim::object::URDF,
          false
      );
      ball->setGeneralizedCoordinate(
          {i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5 - 3 * benchmark::rolling::params.initPenetration,
           1, 0, 0, 0}
      );
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
    sim->cameraFollowObject(checkerboard, {30, 0, 15});
  }
};

double simulationLoop(bool timer = true, bool error = true) {

  // force
  Eigen::VectorXd force(6);
  if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_Y) {
    force << 0, benchmark::rolling::params.F, 0,
        0, 0, 0;
  }
  else if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY) {
    force << benchmark::rolling::params.F * 0.5, benchmark::rolling::params.F * 0.866025403784439, 0,
        0, 0, 0;
  }

  // resever error vector
  benchmark::rolling::data.setN(unsigned(benchmark::rolling::params.T / benchmark::rolling::options.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt); i++) {
    // gui
    if(benchmark::rolling::options.gui && !sim->visualizerLoop(benchmark::rolling::options.dt))
      break;

    // set force to box
    objList[0]->setGeneralizedForce(force);

    // data save
    if(error) {
      benchmark::rolling::data.boxVel.push_back(objList[0]->getGeneralizedVelocity().head(3));
      benchmark::rolling::data.boxPos.push_back(objList[0]->getGeneralizedCoordinate().head(3));
      benchmark::rolling::data.ballVel.push_back(objList[1]->getGeneralizedVelocity().head(3));
      benchmark::rolling::data.ballPos.push_back(objList[1]->getGeneralizedCoordinate().head(3));
    }

    // step
    sim->integrate();
  }

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
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
                << "Num iter : " << benchmark::rolling::options.numSolverIter << std::endl
                << "Solver   : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::rolling::data.computeError();

  // reset
  objList.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);

  // logging
  if(benchmark::rolling::options.csv)
    benchmark::rolling::printCSV(benchmark::rolling::getCSVpath(),
                                 benchmark::bulletmultibody::options.simName,
                                 benchmark::bulletmultibody::options.solverName,
                                 benchmark::bulletmultibody::options.detectorName,
                                 benchmark::bulletmultibody::options.integratorName,
                                 time,
                                 error);

  RAIINFO(
      std::endl << "CPU time   : " << time << std::endl
                << "mean error : " << error << std::endl
                << "speed (Hz) : " << benchmark::rolling::params.T / benchmark::rolling::options.dt / time << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;
}