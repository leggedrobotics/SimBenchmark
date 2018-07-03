//
// Created by kangd on 15.02.18.
//

#include "BtMbSim.hpp"

#include "Elastic666Benchmark.hpp"
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
      if (dist < benchmark::elasticsixsixsix::params.ballR * 2)
        error += (benchmark::elasticsixsixsix::params.ballR * 2 - dist) * (benchmark::elasticsixsixsix::params.ballR * 2 - dist);
    }

    // error sphere ~ ground
    if (objList[i]->getGeneralizedCoordinate()[2] < benchmark::elasticsixsixsix::params.ballR) {
      error +=
          pow(benchmark::elasticsixsixsix::params.ballR - objList[i]->getGeneralizedCoordinate()[2], 2);
    }
  }
  return error;
}

double computeEnergy() {
  double energy = 0;
  for(int j = 0; j < objList.size(); j++)
    energy += objList[j]->getEnergy({0, 0, benchmark::elasticsixsixsix::params.g});
  return energy;
};

void setupSimulation() {
  if (benchmark::elasticsixsixsix::options.gui)
    sim = new bullet_mb_sim::BtMbSim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  else
    sim = new bullet_mb_sim::BtMbSim();

  // erp
  if(benchmark::elasticsixsixsix::options.erpYN)
    sim->setERP(benchmark::elasticsixsixsix::params.erp,
                benchmark::elasticsixsixsix::params.erp2,
                benchmark::elasticsixsixsix::params.erpFriction);
  else
    sim->setERP(0, 0, 0);

  // timestep
  sim->setTimeStep(benchmark::elasticsixsixsix::options.dt);
}

void setupWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::elasticsixsixsix::params.g});

  // random number generator
  rai::RandomNumberGenerator<double> rand;
  rand.seed(benchmark::elasticsixsixsix::params.randomSeed);

  auto checkerboard = sim->addArticulatedSystem(
      benchmark::elasticsixsixsix::getBulletPlanePath(),
      bullet_mb_sim::object::URDF
  );
  checkerboard->setFrictionCoefficient(-1, 0);
  checkerboard->setRestitutionCoefficient(-1, 1.0);

  for(int i = 0; i < benchmark::elasticsixsixsix::params.n; i++) {
    for(int j = 0; j < benchmark::elasticsixsixsix::params.n; j++) {
      for(int k = 0; k < benchmark::elasticsixsixsix::params.n; k++) {

        // add object
        auto obj = sim->addArticulatedSystem(
            benchmark::elasticsixsixsix::getBulletBallPath(),
            bullet_mb_sim::object::URDF
        );

        // set position
        double x =
            double(i) * benchmark::elasticsixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation -0.6;
        double y =
            double(j) * benchmark::elasticsixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation;
        double z =
            double(k) * benchmark::elasticsixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::elasticsixsixsix::params.perturbation
                + benchmark::elasticsixsixsix::params.H;

        obj->setGeneralizedCoordinate({x, y, z, 1, 0, 0, 0});
        obj->setFrictionCoefficient(-1, 0);
        obj->setRestitutionCoefficient(-1, 1.0);

        if(benchmark::elasticsixsixsix::options.gui) {
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

  if(benchmark::elasticsixsixsix::options.gui) {
    sim->setLightPosition((float)benchmark::elasticsixsixsix::params.lightPosition[0],
                          (float)benchmark::elasticsixsixsix::params.lightPosition[1],
                          (float)benchmark::elasticsixsixsix::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {0, 5, 2});
  }
}

double simulationLoop(bool timer = true, bool error = true) {
  // gui
  if(benchmark::elasticsixsixsix::options.gui && benchmark::elasticsixsixsix::options.saveVideo)
    sim->startRecordingVideo("/tmp", "bullet-666");

  // resever error vector
  benchmark::elasticsixsixsix::data.setN(unsigned(benchmark::elasticsixsixsix::options.T / benchmark::elasticsixsixsix::options.dt));

  // timer start
  StopWatch watch;
  if(timer)
    watch.start();

  for(int i = 0; i < (int) (benchmark::elasticsixsixsix::options.T / benchmark::elasticsixsixsix::options.dt); i++) {
    if (benchmark::elasticsixsixsix::options.gui && !sim->visualizerLoop(benchmark::elasticsixsixsix::options.dt))
      break;

    // data save
    if (error) {
      static double E0 = 0;
      if(i==0)
        E0 = computeEnergy();

      double error = pow(computeEnergy() - E0, 2);
      benchmark::elasticsixsixsix::data.error.push_back(error);
    }

    sim->integrate();
  }

  if(benchmark::elasticsixsixsix::options.saveVideo)
    sim->stopRecordingVideo();

  double time = 0;
  if(timer)
    time = watch.measure();
  return time;
}

int main(int argc, const char* argv[]) {

  benchmark::elasticsixsixsix::addDescToOption(desc);
  benchmark::bulletmultibody::addDescToOption(desc);

  benchmark::elasticsixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::bulletmultibody::getOptionsFromArg(argc, argv, desc);

  benchmark::elasticsixsixsix::getParamsFromYAML(benchmark::elasticsixsixsix::getYamlpath().c_str(),
                                                 benchmark::BULLET);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: BULLET" << std::endl
                << "GUI      : " << benchmark::elasticsixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::elasticsixsixsix::options.erpYN << std::endl
                << "Timestep : " << benchmark::elasticsixsixsix::options.dt << std::endl
                << "Solver   : " << benchmark::bulletmultibody::options.solverName << std::endl
                << "-----------------------"
  )

  // trial1: get Error
  setupSimulation();
  setupWorld();
  simulationLoop(false, true);
  double error = benchmark::elasticsixsixsix::data.computeError();

  // reset
  objList.clear();
  delete sim;

  // trial2: get CPU time
  setupSimulation();
  setupWorld();
  double time = simulationLoop(true, false);


  if(benchmark::elasticsixsixsix::options.csv)
    benchmark::elasticsixsixsix::printCSV(benchmark::elasticsixsixsix::getCSVpath(),
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
