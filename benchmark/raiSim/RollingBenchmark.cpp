//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>
#include <valarray>

#include "RollingBenchmark.hpp"

rai_sim::World_RG *sim;
std::vector<rai_sim::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::rolling::options.gui)
    sim = new rai_sim::World_RG(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG();

  // erp
  if(benchmark::rolling::options.erpYN)
    sim->setERP(benchmark::rolling::params.erp);
  else
    sim->setERP(0);

  // time step
  sim->setTimeStep(benchmark::rolling::options.dt);

  if(!benchmark::rolling::options.defaultParam)
    sim->setContactSolverParam(1.0, 0.7, 1.0,
                               benchmark::rolling::options.numSolverIter,
                               benchmark::rolling::options.solverTol);
}

void setupWorld() {

  // materials
  rai_sim::MaterialManager materials;
  materials.setMaterialNames({"ground", "box", "ball"});
  materials.setMaterialPairProp("ground", "ball",
                                benchmark::rolling::params.raiGroundMu * benchmark::rolling::params.raiBallMu,
                                0.0, 0.01);
  materials.setMaterialPairProp("ground", "box",
                                benchmark::rolling::params.raiGroundMu * benchmark::rolling::params.raiBoxMu,
                                0.0, 0.01);
  materials.setMaterialPairProp("ball", "box",
                                benchmark::rolling::params.raiBallMu * benchmark::rolling::params.raiBoxMu,
                                0.0, 0.01);
  sim->updateMaterialProp(materials);

  // add objects
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1, -1, rai_sim::GRID);
  checkerboard->setMaterial(sim->getMaterialKey("ground"));

  auto box = sim->addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5 - benchmark::rolling::params.initPenetration);
  box->setMaterial(sim->getMaterialKey("box"));
  objList.push_back(box);

  if(benchmark::rolling::options.gui)
    box.visual()[0]->setColor({0.5373, 0.6471, 0.3059});

  for(int i = 0; i < benchmark::rolling::params.n; i++) {
    for(int j = 0; j < benchmark::rolling::params.n; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0,
                        j * 2.0 - 4.0,
                        1.5 - 3 * benchmark::rolling::params.initPenetration);
      ball->setMaterial(sim->getMaterialKey("ball"));
      objList.push_back(ball);

      if(benchmark::rolling::options.gui)
        ball.visual()[0]->setColor({0.5373, 0.6471, 0.3059});
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

double simulationLoop(bool timer = true, bool error = true) {

  // force
  rai_sim::Vec<3> force;
  if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_Y)
    force = {0,
             benchmark::rolling::params.F,
             0};
  else if(benchmark::rolling::options.forceDirection == benchmark::rolling::FORCE_XY)
    force = {benchmark::rolling::params.F * 0.5,
             benchmark::rolling::params.F * 0.866025403784439,
             0};

  // resever error vector
  benchmark::rolling::data.setN(unsigned(benchmark::rolling::params.T / benchmark::rolling::options.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  for(int i = 0; i < (int) (benchmark::rolling::params.T / benchmark::rolling::options.dt); i++) {
    // gui
    if(benchmark::rolling::options.gui && !sim->visualizerLoop())
      break;

    // set force to box
    objList[0]->setExternalForce(force, 0);

    // data save
    if(error) {
      benchmark::rolling::data.boxVel.push_back(objList[0]->getLinearVelocity());
      benchmark::rolling::data.boxPos.push_back(objList[0]->getPosition());
      benchmark::rolling::data.ballVel.push_back(objList[1]->getLinearVelocity());
      benchmark::rolling::data.ballPos.push_back(objList[1]->getPosition());
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
  benchmark::rolling::getOptionsFromArg(argc, argv, desc);
  benchmark::rolling::getParamsFromYAML(benchmark::rolling::getYamlpath().c_str(),
                                        benchmark::RAI);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::rolling::options.gui << std::endl
                << "ERP      : " << benchmark::rolling::options.erpYN << std::endl
                << "Force    : " << benchmark::rolling::options.forceDirection << std::endl
                << "Timestep : " << benchmark::rolling::options.dt << std::endl
                << "Num iter : " << benchmark::rolling::options.numSolverIter << std::endl
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

  if(benchmark::rolling::options.csv)
    benchmark::rolling::printCSV(benchmark::rolling::getCSVpath(),
                                 "RAI",
                                 "RAI",
                                 "RAI",
                                 "RAI",
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

