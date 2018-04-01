//
// Created by kangd on 15.02.18.
//

#include <raiSim/World_RG.hpp>

#include "rolling.hpp"
#include <valarray>

namespace rb = rolling_benchmark;

// sim
rai_sim::World_RG *sim;

// functions
void getParams(int argc, const char* argv[], char* yamlfile);
void simulationSetup();

// variables
std::vector<rai_sim::SingleBodyHandle> objectList;

int main(int argc, const char* argv[]) {

  // get parameter from argument and yaml
  getParams(argc, argv, "./rolling.yaml");

  // set up logger and timer
  std::string parentDir =
      rb::parentDir + "-" +
          "erp" + "=" + std::to_string(rb::options.erpYN) + "-" +
          "dir" + "=" + std::to_string(rb::options.forceDirection) + "/";
  rb::loggerSetup(
      benchmark::dataPath + parentDir + "rai",
      std::to_string(rb::params.dt));

  // set up simulation
  simulationSetup();

  rai_sim::Vec<3> force;
  if(rb::options.forceDirection == rb::FORCE_Y)
    force = {0, rb::params.F, 0};
  else if(rb::options.forceDirection == rb::FORCE_XY)
    force = {rb::params.F * 0.707106781186547,
             rb::params.F * 0.707106781186547,
             0};
  std::vector<double> squaredError;

  // simulation loop
  // press 'q' key to quit
  ru::timer->startTimer("rolling");
  if(rb::options.visualize) {
    for(int i = 0; i < rb::params.T / rb::params.dt && sim->visualizerLoop(rb::params.dt); i++) {
      objectList[0]->setExternalForce(force, 0);

      // log
      ru::logger->appendData("velbox", objectList[0]->getLinearVelocity().data());
      ru::logger->appendData("velball", objectList[1]->getLinearVelocity().data());
      ru::logger->appendData("posbox", objectList[0]->getPosition().data());
      ru::logger->appendData("posball", objectList[1]->getPosition().data());
      sim->integrate(rb::params.dt);
    }
  }
  else {
    for(int i = 0; i < rb::params.T / rb::params.dt; i++) {
      objectList[0]->setExternalForce(force, 0);

      // log
      ru::logger->appendData("velbox", objectList[0]->getLinearVelocity().data());
      ru::logger->appendData("velball", objectList[1]->getLinearVelocity().data());
      ru::logger->appendData("posbox", objectList[0]->getPosition().data());
      ru::logger->appendData("posball", objectList[1]->getPosition().data());
      sim->integrate(rb::params.dt);
      squaredError.push_back(std::pow(objectList[1]->getLinearVelocity()[1] - (0.15084944665*(rb::params.dt*(i+1))),2)+
        std::pow(objectList[1]->getLinearVelocity()[0] - (0.15084944665*(rb::params.dt*(i+1))),2)+
        std::pow(objectList[1]->getLinearVelocity()[2],2));
    }
  }
  ru::timer->stopTimer("rolling");

  std::valarray<double> squaredErrorVal{squaredError.data(), squaredError.size()};
  std::cout << "sum = " << squaredErrorVal.sum()/squaredError.size() << "\n";
  // delete sim
  delete sim;

  // time log
  ru::timer->dumpToStdOuput();

  return 0;
}

void simulationSetup() {

  if(rb::options.visualize)
    sim = new rai_sim::World_RG(800, 600, 0.5, rai_sim::NO_BACKGROUND);
  else
    sim = new rai_sim::World_RG;

  sim->setGravity(Eigen::Vector3d(0, 0, rb::params.g));
  if(rb::options.erpYN)
    sim->setERP(rb::params.erp);

  rai_sim::MaterialManager materials;

  // solver params
  sim->setContactSolverParam(1.0, 1, 1.0, 50, 1e-7*rb::params.dt);

  // materials
  materials.setMaterialNames({"ground", "box", "ball"});
  materials.setMaterialPairProp("ground", "ball", rb::params.groundMu * rb::params.ballMu, 0.0, 0.01);
  materials.setMaterialPairProp("ground", "box", rb::params.groundMu * rb::params.boxMu, 0.0, 0.01);
  materials.setMaterialPairProp("ball", "box", rb::params.ballMu * rb::params.boxMu, 0.0, 0.01);
  sim->updateMaterialProp(materials);

  // add ground
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setMaterial(sim->getMaterialKey("ground"));

  // visualization settings
  if(rb::options.visualize) {
    // camera relative position
    sim->setLightPosition(rb::params.lightPosition[0],
                          rb::params.lightPosition[1],
                          rb::params.lightPosition[2]);
    sim->cameraFollowObject(checkerboard, {30, 0, 15});
  }

  // add objects
  auto box = sim->addBox(20, 20, 1, 10);
  box->setPosition(0, 0, 0.5 - rb::params.initPenetration);
  box->setMaterial(sim->getMaterialKey("box"));
  objectList.push_back(box);

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, 1.5 - 3 * rb::params.initPenetration);
      ball->setMaterial(sim->getMaterialKey("ball"));
      objectList.push_back(ball);
    }
  }
}


void getParams(int argc, const char *argv[], char *yamlfile) {

  /// parameters from yaml
  YAML::Node yaml = YAML::LoadFile(yamlfile);

  // sim specific
  rb::params.erp = yaml["solver_params"]["raiSim"]["erp"].as<double>();

  // generic
  rb::getParamsFromYAML(yamlfile);

  /// parameter from arguments
  // sim specific
  po::options_description simdesc("sim specific");
  rb::desc.add(simdesc);

  // generic
  rb::getParamsFromArg(argc, argv);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, rb::desc), vm);

  RAIINFO("----------------------")
  RAIINFO("raiSim")
  RAIINFO("timestep        = " << rb::params.dt);
  RAIINFO("erpYN           = " << rb::options.erpYN);
  RAIINFO("force-direction = " << rb::options.forceDirection);
}
