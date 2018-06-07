//
// Created by kangd on 15.02.18.
//

#include "BtMbSim.hpp"

#include "666Benchmark.hpp"
#include "BtMbBenchmark.hpp"

bullet_mb_sim::BtMbSim *sim;
std::vector<bullet_mb_sim::ArticulatedSystemHandle> objList;
po::options_description desc;

double penetrationCheck() {
  double error = 0;
  int numObj = objList.size();

  for (int i = 0; i < numObj; i++) {
    for (int j = i + 1; j < numObj; j++) {
      double dist = (objList[i]->getGeneralizedCoordinate().head(3) - objList[j]->getGeneralizedCoordinate().head(3)).norm();

      // error between spheres
      if (dist < benchmark::sixsixsix::params.ballR * 2)
        error += (benchmark::sixsixsix::params.ballR * 2 - dist) * (benchmark::sixsixsix::params.ballR * 2 - dist);
    }

    // error sphere ~ ground
    if (objList[i]->getGeneralizedCoordinate()[2] < benchmark::sixsixsix::params.ballR) {
      error +=
          pow(benchmark::sixsixsix::params.ballR - objList[i]->getGeneralizedCoordinate()[2], 2);
    }
  }
  return error;
}

double computeEnergy() {
  double energy = 0;
  for(int j = 0; j < objList.size(); j++)
    energy += objList[j]->getEnergy({0, 0, benchmark::sixsixsix::params.g});
  return energy;
};

void setupSimulation() {
  if (benchmark::sixsixsix::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // erp
  if(benchmark::sixsixsix::options.erpYN)
    sim->setERP(benchmark::sixsixsix::params.erp,
                benchmark::sixsixsix::params.erp2,
                benchmark::sixsixsix::params.erpFriction);
  else
    sim->setERP(0, 0, 0);

  // timestep
  sim->setTimeStep(benchmark::sixsixsix::options.dt);
}

void setupWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::sixsixsix::params.g});

  // random number generator
  rai::RandomNumberGenerator<double> rand;
  rand.seed(benchmark::sixsixsix::params.randomSeed);

  auto checkerboard = sim->addArticulatedSystem(
      benchmark::sixsixsix::getBulletPlanePath(),
      bullet_mb_sim::object::URDF
  );
  checkerboard->setFrictionCoefficient(-1, 0);
  if(benchmark::sixsixsix::options.elasticCollision)
    checkerboard->setRestitutionCoefficient(-1, 1.0);

  for(int i = 0; i < benchmark::sixsixsix::params.n; i++) {
    for(int j = 0; j < benchmark::sixsixsix::params.n; j++) {
      for(int k = 0; k < benchmark::sixsixsix::params.n; k++) {

        // add object
        auto obj = sim->addArticulatedSystem(
            benchmark::sixsixsix::getBulletBallPath(),
            bullet_mb_sim::object::URDF
        );

        // set position
        double x =
            double(i) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation -0.6;
        double y =
            double(j) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation;
        double z =
            double(k) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation
                + benchmark::sixsixsix::params.H;

        obj->setGeneralizedCoordinate({x, y, z, 1, 0, 0, 0});
        obj->setFrictionCoefficient(-1, 0);
        if(benchmark::sixsixsix::options.elasticCollision)
          obj->setRestitutionCoefficient(-1, 1.0);

        if(benchmark::sixsixsix::options.gui) {
          if((i + j + k) % 3 == 0) {
            obj.visual()[0]->setColor({benchmark::bulletmultibody::color[0],
                                       benchmark::bulletmultibody::color[1],
                                       benchmark::bulletmultibody::color[2]});
          }
          else if((i + j + k) % 3 == 1) {
            obj.visual()[0]->setColor({benchmark::bulletmultibody::color[0],
                                       benchmark::bulletmultibody::color[1],
                                       benchmark::bulletmultibody::color[2]});
          }
          else if((i + j + k) % 3 == 2) {
            obj.visual()[0]->setColor({benchmark::bulletmultibody::color[0],
                                       benchmark::bulletmultibody::color[1],
                                       benchmark::bulletmultibody::color[2]});
          }
        }

        objList.push_back(obj);
      }
    }
  }

  if(benchmark::sixsixsix::options.gui) {
    sim->setLightPosition((float)benchmark::sixsixsix::params.lightPosition[0],
                          (float)benchmark::sixsixsix::params.lightPosition[1],
                          (float)benchmark::sixsixsix::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {0, 5, 2});
  }
}

double simulationLoop(bool timer = true, bool error = true) {
  // gui
  if(benchmark::sixsixsix::options.gui && benchmark::sixsixsix::options.saveVideo)
    sim->startRecordingVideo("/tmp", "bullet-666");

  // resever error vector
  benchmark::sixsixsix::data.setN(unsigned(benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt); i++) {
    if (benchmark::sixsixsix::options.gui && !sim->visualizerLoop(benchmark::sixsixsix::options.dt))
      break;

    // data save
    if (error) {
      static double E0 = 0;
      if(i==0) 
        E0 = computeEnergy();

      if (benchmark::sixsixsix::options.elasticCollision) {
        double error = pow(computeEnergy() - E0, 2);
        benchmark::sixsixsix::data.error.push_back(error);
      }
      else {
        double error = penetrationCheck();
        benchmark::sixsixsix::data.error.push_back(error);
      }
    }

    sim->integrate();
  }

  if(benchmark::sixsixsix::options.saveVideo)
    sim->stopRecordingVideo();
  
  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::sixsixsix::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::sixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::sixsixsix::getParamsFromYAML(benchmark::sixsixsix::getYamlpath().c_str(),
                                          benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: BULLET" << std::endl
                << "GUI      : " << benchmark::sixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::sixsixsix::options.erpYN << std::endl
                << "Elastic  : " << benchmark::sixsixsix::options.elasticCollision << std::endl
                << "Timestep : " << benchmark::sixsixsix::options.dt << std::endl
                << "Solver   : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::sixsixsix::data.computeError();

  // reset
  objList.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);


  if(benchmark::sixsixsix::options.csv)
    benchmark::sixsixsix::printCSV(benchmark::sixsixsix::getCSVpath(),
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
