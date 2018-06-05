//
// Created by kangd on 15.02.18.
//

#include <BtSim.hpp>

#include "BouncingBenchmark.hpp"
#include "BtBenchmark.hpp"

// sim
bullet_sim::BtSim *sim;

// objects
std::vector<benchmark::SingleBodyHandle> objList;

// description
po::options_description desc;

void setupSimulation() {
  if (benchmark::bouncing::options.gui)
    sim = new bullet_sim::BtSim(800, 600, 0.5, benchmark::bullet::options.solverOption, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_sim::BtSim(benchmark::bullet::options.solverOption);


  // erp
  if(benchmark::bouncing::options.erpYN)
    sim->setERP(benchmark::bouncing::params.erp, 0, 0);
  else
    sim->setERP(0, 0, 0);
}

void setupWorld() {
  // materials
  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, bo::BOX_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(benchmark::bouncing::params.mu_ground);
  checkerboard->setRestitutionCoefficient(1.0);

  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      auto ball = sim->addSphere(benchmark::bouncing::params.R, benchmark::bouncing::params.m);
      ball->setPosition(i * 2.0 - 10, j * 2.0 - 10, benchmark::bouncing::params.H);
      ball->setFrictionCoefficient(benchmark::bouncing::params.mu_ball);
      ball->setRestitutionCoefficient(benchmark::bouncing::options.e);

      if(benchmark::bouncing::options.gui)
        ball.visual()[0]->setColor(
            {benchmark::bullet::color[0],
             benchmark::bullet::color[1],
             benchmark::bullet::color[2]});

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

double simulationLoop(bool timer = true, bool error = true) {
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

    sim->integrate(benchmark::bouncing::options.dt);
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
  benchmark::bullet::addDescToOption(desc);

  benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
  benchmark::bullet::getOptionsFromArg(argc, argv, desc);

  benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlPath().c_str(),
                                         benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: BULLET" << std::endl
                << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                << "Timestep : " << benchmark::bouncing::options.dt << std::endl
                << "Solver   : " << benchmark::bullet::options.solverName << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::bouncing::data.computeError();

  // reset
  objList.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);

  if(benchmark::bouncing::options.csv)
    benchmark::bouncing::printCSV(benchmark::bouncing::getCSVpath(),
                                  benchmark::bullet::options.simName,
                                  benchmark::bullet::options.solverName,
                                  benchmark::bullet::options.detectorName,
                                  benchmark::bullet::options.integratorName,
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