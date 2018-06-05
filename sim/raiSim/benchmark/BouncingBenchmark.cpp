//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "BouncingBenchmark.hpp"

rai_sim::World_RG *sim;
std::vector<rai_sim::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::bouncing::options.gui)
    sim = new rai_sim::World_RG(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG();

  // erp
  if(benchmark::bouncing::options.erpYN)
    sim->setERP(benchmark::bouncing::params.erp);
  else
    sim->setERP(0);
}

void setupWorld() {
  // materials
  rai_sim::MaterialManager materials;
  materials.setMaterialNames({"ground", "ball"});
  materials.setMaterialPairProp("ground", "ball",
                                benchmark::bouncing::params.mu_ground * benchmark::bouncing::params.mu_ball,
                                benchmark::bouncing::options.e,
                                0);
  sim->updateMaterialProp(materials);

  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, 1, -1, rai_sim::GRID);
  checkerboard->setMaterial(sim->getMaterialKey("ground"));

  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      auto ball = sim->addSphere(benchmark::bouncing::params.R, benchmark::bouncing::params.m);
      ball->setPosition(i * 2.0, j * 2.0, benchmark::bouncing::params.H);
      ball->setMaterial(sim->getMaterialKey("ball"));

      if(benchmark::bouncing::options.gui)
        ball.visual()[0]->setColor({0.5373, 0.6471, 0.3059});

      objList.push_back(ball);
    }
  }

  // gravity
  sim->setGravity({0, 0, benchmark::bouncing::params.g});

  if(benchmark::bouncing::options.gui) {
    sim->setLightPosition((float)benchmark::bouncing::params.lightPosition[0],
                          (float)benchmark::bouncing::params.lightPosition[1],
                          (float)benchmark::bouncing::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {10, 0, 10});
  }
}

void resetWorld() {
  int cnt = 0;
  for(int i = 0; i < benchmark::bouncing::params.n; i++) {
    for(int j = 0; j < benchmark::bouncing::params.n; j++) {
      objList[cnt]->setPosition(
          i * 2.0 - 10,
          j * 2.0 - 10,
          benchmark::bouncing::params.H
      );
      objList[cnt++]->setVelocity(
          0, 0, 0, 0, 0, 0
      );
    }
  }
}

double simulationLoop(bool timer = true, bool error = true) {
  if(benchmark::bouncing::options.saveVideo)
    sim->startRecordingVideo("/tmp", "ode-bouncing");

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
  benchmark::bouncing::getOptionsFromArg(argc, argv, desc);
  benchmark::bouncing::getParamsFromYAML(benchmark::bouncing::getYamlPath().c_str(),
                                         benchmark::RAI);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::bouncing::options.gui << std::endl
                << "ERP      : " << benchmark::bouncing::options.erpYN << std::endl
                << "Res Coef : " << benchmark::bouncing::options.e << std::endl
                << "Timestep : " << benchmark::bouncing::options.dt << std::endl
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
                                  "RAI",
                                  "RAI",
                                  "RAI",
                                  "RAI",
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