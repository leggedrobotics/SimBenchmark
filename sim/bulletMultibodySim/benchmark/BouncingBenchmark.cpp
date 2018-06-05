//
// Created by kangd on 15.02.18.
//

#include <BtMbSim.hpp>

#include "BouncingBenchmark.hpp"
#include "BtMbBenchmark.hpp"

// sim
bullet_mb_sim::BtMbSim *sim;

// objects
std::vector<bullet_mb_sim::ArticulatedSystemHandle> objList;

// description
po::options_description desc;

void setupSimulation() {
  if (benchmark::bouncing::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // erp
  if(benchmark::bouncing::options.erpYN)
    sim->setERP(benchmark::bouncing::params.erp, 0, 0);
  else
    sim->setERP(0, 0, 0);

  // timestep
  sim->setTimeStep(benchmark::bouncing::options.dt);
}

void setupWorld() {
  // materials
  // add objects
  auto checkerboard = sim->addArticulatedSystem(
      benchmark::bouncing::getBulletPlanepath(),
      bullet_mb_sim::object::URDF
  );
  checkerboard->setFrictionCoefficient(-1, benchmark::bouncing::params.mu_ground);
  checkerboard->setRestitutionCoefficient(-1, 1);

  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      auto ball = sim->addArticulatedSystem(
          benchmark::bouncing::getBulletBallPath(),
          bullet_mb_sim::object::URDF
      );
      ball->setGeneralizedCoordinate(
          {i * 2.0 - 10,
           j * 2.0 - 10,
           benchmark::bouncing::params.H,
           1, 0, 0, 0});
      ball->setFrictionCoefficient(-1, benchmark::bouncing::params.mu_ball);
      ball->setRestitutionCoefficient(-1, benchmark::bouncing::options.e);

      if(benchmark::bouncing::options.gui)
        ball.visual()[0]->setColor(
            {benchmark::bulletmultibody::color[0],
             benchmark::bulletmultibody::color[1],
             benchmark::bulletmultibody::color[2]});

      objList.push_back(ball);
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::bouncing::params.g});

  if(benchmark::bouncing::options.gui) {
    sim->setLightPosition((float)benchmark::bouncing::params.lightPosition[0],
                          (float)benchmark::bouncing::params.lightPosition[1],
                          (float)benchmark::bouncing::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {10, 0, 6});
  }
}

void resetWorld() {
  int cnt = 0;
  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      objList[cnt]->setGeneralizedCoordinate(
          {i * 2.0 - 10,
           j * 2.0 - 10,
           benchmark::bouncing::params.H,
           1, 0, 0, 0});
      objList[cnt++]->setGeneralizedVelocity(
          {0, 0, 0, 0, 0, 0});
    }
  }
}

double simulationLoop(bool timer = true, bool error = true) {
  // gui
  if(benchmark::bouncing::options.saveVideo)
    sim->startRecordingVideo("/tmp", "bullet-bouncing");

  // resever error vector
  benchmark::bouncing::data.setN(unsigned(benchmark::bouncing::params.T / benchmark::bouncing::options.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  for(int i = 0; i < (int) (benchmark::bouncing::params.T / benchmark::bouncing::options.dt); i++) {
    // gui
    if (benchmark::bouncing::options.gui && !sim->visualizerLoop(benchmark::bouncing::options.dt))
      break;

    // data save
    if (error) {
      double E = 0;
      for(int j = 0; j < objList.size(); j++) {
        E += objList[j]->getEnergy({0, 0, benchmark::bouncing::params.g});
      }
      benchmark::bouncing::data.ballEnergy.push_back(E);
    }

    sim->integrate();
  }

  if(benchmark::bouncing::options.saveVideo)
    sim->stopRecordingVideo();

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::bouncing::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlPath().c_str(),
                                         benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: BULLET" << std::endl
                << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                << "Timestep : " << benchmark::bouncing::options.dt << std::endl
                << "Solver   : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "-----------------------"
  )

  // set-up
  setupSimulation();
  setupWorld();

  // trial1: get Error
  resetWorld();
  simulationLoop(false, true);
  double error = benchmark::bouncing::data.computeError();

  // trial2: get CPU time
  resetWorld();
  double time = simulationLoop(true, false);

  if(benchmark::bouncing::options.csv)
    benchmark::bouncing::printCSV(benchmark::bouncing::getCSVpath(),
                                  benchmark::bulletmultibody::options.simName,
                                  benchmark::bulletmultibody::options.solverName,
                                  benchmark::bulletmultibody::options.detectorName,
                                  benchmark::bulletmultibody::options.integratorName,
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