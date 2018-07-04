//
// Created by kangd on 26.04.18.
//

#include <BtMbSim.hpp>

#include "AtlasContactBenchmark.hpp"
#include "BtMbBenchmark.hpp"
#include "raiCommon/utils/StopWatch.hpp"

bullet_mb_sim::BtMbSim *sim;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> robots;
po::options_description desc;

void setupSimulation() {
  if(benchmark::atlas::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // time step
  sim->setTimeStep(benchmark::atlas::params.dt);
  sim->setERP(0.05, 0.01, 0.1);
}

void resetWorld() {
  auto checkerboard = sim->addArticulatedSystem(benchmark::atlas::getBulletPlanePath(),
                                                bullet_mb_sim::object::URDF,
                                                true,
                                                benchmark::bulletmultibody::options.maximalCoordinate);

  checkerboard->setFrictionCoefficient(-1, 1.0);  // 1.0(ground) x 0.8(feet) = 0.8
  for(int i = 0; i < benchmark::atlas::options.numRow; i++) {
    for(int j = 0; j < benchmark::atlas::options.numRow; j++) {
      auto robot =
          sim->addArticulatedSystem(benchmark::atlas::getBulletAtlasPath(),
                                    bullet_mb_sim::object::URDF,
                                    true,
                                    benchmark::bulletmultibody::options.maximalCoordinate);
//      atlas->setColor({1, 0, 0, 1});

      Eigen::VectorXd gc(robot->getStateDimension());
      Eigen::VectorXd gv(robot->getDOF());
      Eigen::VectorXd tau(robot->getDOF());
      gc.setZero();
      gc.segment<7>(0) << 2.5 * i, 2.5 * j, benchmark::atlas::params.H,
          benchmark::atlas::params.baseQuat[0],
          benchmark::atlas::params.baseQuat[1],
          benchmark::atlas::params.baseQuat[2],
          benchmark::atlas::params.baseQuat[3];

      robot->setState(gc, gv);
      robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
      robots.push_back(robot);
    }
  }

  sim->setGravity({0, 0, benchmark::atlas::params.g});

  if(benchmark::atlas::options.gui)
    sim->cameraFollowObject(checkerboard, {1.0, 1.0, 1.0});
}

double simulationLoop() {
  int numContact = 0;
  int cnt = 0;
  if(benchmark::atlas::options.gui) {
    // gui
    while(sim->visualizerLoop(benchmark::atlas::params.dt)) {
      sim->integrate();
      numContact += sim->getWorldNumContacts();
      cnt++;
    }
  } else {
    // no gui
    StopWatch watch;
    watch.start();
    for(int t = 0; t < (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt); t++) {
      sim->integrate();
      numContact += sim->getWorldNumContacts();
      cnt++;
    }

    double time = watch.measure();

    // print to screen
    std::cout<<"time taken for "
             << (int) (benchmark::atlas::params.T / benchmark::atlas::params.dt)
             << " steps "<< time <<"s \n";

    if(benchmark::atlas::options.csv)
      benchmark::atlas::printCSV(benchmark::atlas::getCSVpath(),
                                 benchmark::bulletmultibody::options.simName,
                                 benchmark::bulletmultibody::options.solverName,
                                 benchmark::bulletmultibody::options.detectorName,
                                 benchmark::bulletmultibody::options.integratorName,
                                 benchmark::atlas::options.numRow,
                                 double(numContact) / cnt,
                                 time);
  }
  return double(numContact) / cnt;
}

int main(int argc, const char* argv[]) {

  benchmark::atlas::addDescToOption(desc);
  benchmark::atlas::getOptionsFromArg(argc, argv, desc);

  benchmark::atlas::getParamsFromYAML(benchmark::atlas::getYamlpath().c_str(),
                                      benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: " << benchmark::bulletmultibody::options.simName << std::endl
                << "GUI      : " << benchmark::atlas::options.gui << std::endl
                << "Row      : " << benchmark::atlas::options.numRow << std::endl
                << "-----------------------"
  )

  double contact;
  setupSimulation();
  resetWorld();
  contact = simulationLoop();

  RAIINFO(
      std::endl << "-----------------------" << std::endl
                << "Contacts : " << contact << std::endl
                << "=======================" << std::endl
  )

  delete sim;
  return 0;
}