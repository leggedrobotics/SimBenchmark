//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "666Benchmark.hpp"

rai_sim::World_RG *sim;
std::vector<rai_sim::SingleBodyHandle> objList;
po::options_description desc;

void setupSimulation() {
  if (benchmark::sixsixsix::options.gui)
    sim = new rai_sim::World_RG(800, 600, 1, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG();

  // erp
  if(benchmark::sixsixsix::options.erpYN)
    sim->setERP(benchmark::sixsixsix::params.erp);
  else
    sim->setERP(0);
}

void setupWorld() {

  // gravity
  sim->setGravity({0, 0, benchmark::sixsixsix::params.g});

  // materials
  rai_sim::MaterialManager materials;
  materials.setMaterialNames({"ground", "ball"});
  if(benchmark::sixsixsix::options.elasticCollision) {
    materials.setMaterialPairProp("ground", "ball",
                                  0, 1, 0);
    materials.setMaterialPairProp("ball", "ball",
                                  0, 1, 0);
  }
  else {
    materials.setMaterialPairProp("ground", "ball",
                                  0, 0, 0.01);
    materials.setMaterialPairProp("ball", "ball",
                                  0, 0, 0.01);
  }
  sim->updateMaterialProp(materials);

  // random number generator
  rai::RandomNumberGenerator<double> rand;
  rand.seed(benchmark::sixsixsix::params.randomSeed);

  auto checkerboard = sim->addCheckerboard(5.0, 500.0, 500.0, 0.1, 1, -1, rai_sim::GRID);
  checkerboard->setMaterial(sim->getMaterialKey("ground"));

  for(int i = 0; i < benchmark::sixsixsix::params.n; i++) {
    for(int j = 0; j < benchmark::sixsixsix::params.n; j++) {
      for(int k = 0; k < benchmark::sixsixsix::params.n; k++) {

        // add object
        auto obj = sim->addSphere(benchmark::sixsixsix::params.ballR,
                                  benchmark::sixsixsix::params.ballM);

        // set position
        double x =
            double(i) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation;
        double y =
            double(j) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation;
        double z =
            double(k) * benchmark::sixsixsix::params.gap
                + rand.sampleUniform01() * benchmark::sixsixsix::params.perturbation
                + benchmark::sixsixsix::params.H;

        obj->setPosition(x, y, z);
        obj->setMaterial(sim->getMaterialKey("ball"));
//        obj->setOrientationRandom();

        if(benchmark::sixsixsix::options.gui) {
          if((i + j + k) % 3 == 0) {
            obj.visual()[0]->setColor({0.5373,
                                       0.6471,
                                       0.3059});
          }
          else if((i + j + k) % 3 == 1) {
            obj.visual()[0]->setColor({0.5373,
                                       0.6471,
                                       0.3059});
          }
          else if((i + j + k) % 3 == 2) {
            obj.visual()[0]->setColor({0.5373,
                                       0.6471,
                                       0.3059});
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
    sim->cameraFollowObject(objList[objList.size() / 2], {0, 5, 2});
  }
}

double penetrationCheck() {
  double error = 0;
  int numObj = objList.size();

  for (int i = 0; i < numObj; i++) {
    for (int j = i + 1; j < numObj; j++) {
      double dist = (objList[i]->getPosition() - objList[j]->getPosition()).norm();

      // error between spheres
      if (dist < benchmark::sixsixsix::params.ballR * 2)
        error += (benchmark::sixsixsix::params.ballR * 2 - dist) * (benchmark::sixsixsix::params.ballR * 2 - dist);
    }

    // error sphere ~ ground
    if (objList[i]->getPosition()[2] < benchmark::sixsixsix::params.ballR) {
      error +=
          (benchmark::sixsixsix::params.ballR - objList[i]->getPosition()[2]) * (benchmark::sixsixsix::params.ballR - objList[i]->getPosition()[2]);
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

void simulationLoop() {

  // init
  double E0 = 0;
  benchmark::sixsixsix::energyList.reserve(unsigned(benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt));
  benchmark::sixsixsix::errorList.reserve(unsigned(benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt));

  // loop
  StopWatch watch;
  watch.start();
  if(benchmark::sixsixsix::options.gui) {
    // gui
    if(benchmark::sixsixsix::options.saveVideo)
      sim->startRecordingVideo("/tmp", "rai-thousand");

    for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt)
        && sim->visualizerLoop(benchmark::sixsixsix::options.dt); i++) {

      double error = penetrationCheck();
      double energy = computeEnergy();
      if(i==0) E0 = energy;

      benchmark::sixsixsix::energyList.push_back(energy);
      benchmark::sixsixsix::errorList.push_back(error);

      sim->integrate(benchmark::sixsixsix::options.dt);
    }

    if(benchmark::sixsixsix::options.saveVideo)
      sim->stopRecordingVideo();
  }
  else {
    // no gui
    for(int i = 0; i < (int) (benchmark::sixsixsix::options.T / benchmark::sixsixsix::options.dt); i++) {

      double error = penetrationCheck();
      double energy = computeEnergy();
      if(i==0) E0 = energy;

      benchmark::sixsixsix::energyList.push_back(energy);
      benchmark::sixsixsix::errorList.push_back(error);

      sim->integrate(benchmark::sixsixsix::options.dt);
    }
  }
  double time = watch.measure();

  benchmark::sixsixsix::printError(E0);
  if(benchmark::sixsixsix::options.log)
    benchmark::sixsixsix::printCSV(
        benchmark::sixsixsix::getLogFilepath(),
        "RAI",
        "RAI",
        time,
        E0
    );
}

int main(int argc, const char* argv[]) {

  benchmark::sixsixsix::addDescToOption(desc);
  benchmark::sixsixsix::getOptionsFromArg(argc, argv, desc);
  benchmark::sixsixsix::getParamsFromYAML(benchmark::sixsixsix::getYamlpath().c_str(),
                                         benchmark::RAI);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: RAI" << std::endl
                << "GUI      : " << benchmark::sixsixsix::options.gui << std::endl
                << "ERP      : " << benchmark::sixsixsix::options.erpYN << std::endl
                << "Timestep : " << benchmark::sixsixsix::options.dt << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  delete sim;
  return 0;

}
