//
// Created by kangd on 26.04.18.
//

#include <MjcWorld_RG.hpp>

#include "AnymalBenchmark.hpp"
#include "MjcBenchmark.hpp"
#include "raiCommon/utils/StopWatch.hpp"

mujoco_sim::MjcWorld_RG *sim;
po::options_description desc;

void setupSimulation() {
  int numRow = benchmark::anymal::options.numRow;

  // gui
  if(benchmark::anymal::options.gui)
    sim = new mujoco_sim::MjcWorld_RG(800, 600, 0.5,
                                      benchmark::anymal::getMujocoURDFpath(numRow).c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::NO_BACKGROUND,
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);
  else
    sim = new mujoco_sim::MjcWorld_RG(benchmark::anymal::getMujocoURDFpath(numRow).c_str(),
                                      benchmark::mujoco::getKeypath().c_str(),
                                      benchmark::mujoco::options.solverOption,
                                      benchmark::mujoco::options.integratorOption);

  // no slip parameter
  if(benchmark::mujoco::options.noSlip)
    sim->setNoSlipParameter(10);

  // timestep
  sim->setTimeStep(benchmark::anymal::params.dt);
}

void setupWorld() {
  int numRow = benchmark::anymal::options.numRow;
  Eigen::VectorXd genCoord(sim->getStateDimension());

  int cnt = 0;
  for(int i = 0; i < numRow; i++) {
    for (int j = 0; j < numRow; j++) {
      genCoord[cnt * 19 + 0] = i * 2;
      genCoord[cnt * 19 + 1] = j * 2;
      genCoord[cnt * 19 + 2] = benchmark::anymal::params.H;
      genCoord[cnt * 19 + 3] = benchmark::anymal::params.baseQuat[0];
      genCoord[cnt * 19 + 4] = benchmark::anymal::params.baseQuat[1];
      genCoord[cnt * 19 + 5] = benchmark::anymal::params.baseQuat[2];
      genCoord[cnt * 19 + 6] = benchmark::anymal::params.baseQuat[3];
      genCoord[cnt * 19 + 7] = benchmark::anymal::params.jointPos[0];
      genCoord[cnt * 19 + 8] = benchmark::anymal::params.jointPos[1];
      genCoord[cnt * 19 + 9] = benchmark::anymal::params.jointPos[2];
      genCoord[cnt * 19 + 10] = benchmark::anymal::params.jointPos[3];
      genCoord[cnt * 19 + 11] = benchmark::anymal::params.jointPos[4];
      genCoord[cnt * 19 + 12] = benchmark::anymal::params.jointPos[5];
      genCoord[cnt * 19 + 13] = benchmark::anymal::params.jointPos[6];
      genCoord[cnt * 19 + 14] = benchmark::anymal::params.jointPos[7];
      genCoord[cnt * 19 + 15] = benchmark::anymal::params.jointPos[8];
      genCoord[cnt * 19 + 16] = benchmark::anymal::params.jointPos[9];
      genCoord[cnt * 19 + 17] = benchmark::anymal::params.jointPos[10];
      genCoord[cnt * 19 + 18] = benchmark::anymal::params.jointPos[11];
      cnt++;
    }
  }

  sim->setGeneralizedCoordinate(genCoord);
  sim->setGeneralizedVelocity(Eigen::VectorXd::Zero(sim->getDOF()));
  sim->setGeneralizedForce(Eigen::VectorXd::Zero(sim->getDOF()));

  sim->setGravity({0, 0, benchmark::anymal::params.g});

  if(benchmark::anymal::options.gui)
    sim->cameraFollowObject(
        sim->getSingleBodyHandle(0), {1.0, 1.0, 1.0});
}

void simulationLoop() {
  Eigen::VectorXd jointNominalConfig(sim->getStateDimension());
  Eigen::VectorXd jointState(sim->getStateDimension());
  Eigen::VectorXd jointVel(sim->getDOF());
  Eigen::VectorXd jointForce(sim->getDOF());

  const double kp = benchmark::anymal::params.kp;
  const double kd = benchmark::anymal::params.kd;

  //  jointNominalConfig
  int cnt = 0;
  for(int i = 0; i < benchmark::anymal::options.numRow; i++) {
    for (int j = 0; j < benchmark::anymal::options.numRow; j++) {
      jointNominalConfig[cnt * 19 + 0] = 0;
      jointNominalConfig[cnt * 19 + 1] = 0;
      jointNominalConfig[cnt * 19 + 2] = 0;
      jointNominalConfig[cnt * 19 + 3] = 1;
      jointNominalConfig[cnt * 19 + 4] = 0;
      jointNominalConfig[cnt * 19 + 5] = 0;
      jointNominalConfig[cnt * 19 + 6] = 0;
      jointNominalConfig[cnt * 19 + 7] = benchmark::anymal::params.jointPos[0];
      jointNominalConfig[cnt * 19 + 8] = benchmark::anymal::params.jointPos[1];
      jointNominalConfig[cnt * 19 + 9] = benchmark::anymal::params.jointPos[2];
      jointNominalConfig[cnt * 19 + 10] = benchmark::anymal::params.jointPos[3];
      jointNominalConfig[cnt * 19 + 11] = benchmark::anymal::params.jointPos[4];
      jointNominalConfig[cnt * 19 + 12] = benchmark::anymal::params.jointPos[5];
      jointNominalConfig[cnt * 19 + 13] = benchmark::anymal::params.jointPos[6];
      jointNominalConfig[cnt * 19 + 14] = benchmark::anymal::params.jointPos[7];
      jointNominalConfig[cnt * 19 + 15] = benchmark::anymal::params.jointPos[8];
      jointNominalConfig[cnt * 19 + 16] = benchmark::anymal::params.jointPos[9];
      jointNominalConfig[cnt * 19 + 17] = benchmark::anymal::params.jointPos[10];
      jointNominalConfig[cnt * 19 + 18] = benchmark::anymal::params.jointPos[11];
      cnt++;
    }
  }

  sim->setGeneralizedVelocity(Eigen::VectorXd::Zero(sim->getDOF()));
  sim->setGeneralizedForce(Eigen::VectorXd::Zero(sim->getDOF()));

  if(benchmark::anymal::options.gui) {
    // gui
    while(sim->visualizerLoop(benchmark::anymal::params.dt, 1.0)) {

      // joint force
      if(benchmark::anymal::options.feedback) {
        jointState = sim->getGeneralizedCoordinate();
        jointVel = sim->getGeneralizedVelocity();

        int cnt = 0;
        for(int i = 0; i < benchmark::anymal::options.numRow; i++) {
          for (int j = 0; j < benchmark::anymal::options.numRow; j++) {
            jointForce[cnt * 18 + 0] = 0;
            jointForce[cnt * 18 + 1] = 0;
            jointForce[cnt * 18 + 2] = 0;
            jointForce[cnt * 18 + 3] = 0;
            jointForce[cnt * 18 + 4] = 0;
            jointForce[cnt * 18 + 5] = 0;
            for(int k = 6; k < 18; k++)
              jointForce[cnt * 18 + k] =
                  kp * (jointNominalConfig[cnt * 19 + k + 1] - jointState[cnt * 19 + k + 1]) - kd * jointVel[cnt * 18 + k];

            cnt++;
          }
        }
        sim->setGeneralizedForce(jointForce);
      }

      sim->integrate();
    }
  } else {
    // no gui
    StopWatch watch;
    watch.start();
    for(int t = 0; t < (int)(benchmark::anymal::params.T / benchmark::anymal::params.dt); t++) {

      // joint force
      if(benchmark::anymal::options.feedback) {
        jointState = sim->getGeneralizedCoordinate();
        jointVel = sim->getGeneralizedVelocity();

        int cnt = 0;
        for(int i = 0; i < benchmark::anymal::options.numRow; i++) {
          for (int j = 0; j < benchmark::anymal::options.numRow; j++) {
            jointForce[cnt * 18 + 0] = 0;
            jointForce[cnt * 18 + 1] = 0;
            jointForce[cnt * 18 + 2] = 0;
            jointForce[cnt * 18 + 3] = 0;
            jointForce[cnt * 18 + 4] = 0;
            jointForce[cnt * 18 + 5] = 0;
            for(int k = 6; k < 18; k++)
              jointForce[cnt * 18 + k] =
                  kp * (jointNominalConfig[cnt * 19 + k + 1] - jointState[cnt * 19 + k + 1]) - kd * jointVel[cnt * 18 + k];

            cnt++;
          }
        }
        sim->setGeneralizedForce(jointForce);
      }

      sim->integrate();
    }

    double time = watch.measure();

    // print to screen
    std::cout<<"time taken for "
             << (int) (benchmark::anymal::params.T / benchmark::anymal::params.dt)
             << " steps "<< time <<"s \n";

    if(benchmark::anymal::options.csv)
      benchmark::anymal::printCSV(benchmark::anymal::getCSVpath(benchmark::anymal::options.feedback),
                                  benchmark::mujoco::options.simName,
                                  benchmark::mujoco::options.solverName,
                                  benchmark::mujoco::options.detectorName,
                                  benchmark::mujoco::options.integratorName,
                                  benchmark::anymal::options.numRow,
                                  time);
  }
}

int main(int argc, const char* argv[]) {

  benchmark::anymal::addDescToOption(desc);
  benchmark::mujoco::addDescToOption(desc);

  benchmark::anymal::getOptionsFromArg(argc, argv, desc);
  benchmark::mujoco::getOptionsFromArg(argc, argv, desc);

  benchmark::anymal::getParamsFromYAML(benchmark::anymal::getYamlpath().c_str(),
                                       benchmark::MUJOCO);

  RAIINFO(
      std::endl << "=======================" << std::endl
                << "Simulator: MUJOCO" << std::endl
                << "GUI      : " << benchmark::anymal::options.gui << std::endl
                << "Row      : " << benchmark::anymal::options.numRow << std::endl
                << "Feedback : " << benchmark::anymal::options.feedback << std::endl
                << "Solver   : " << benchmark::mujoco::options.solverOption << std::endl
                << "No Slip  : " << benchmark::mujoco::options.noSlip << std::endl
                << "-----------------------"
  )

  setupSimulation();
  setupWorld();
  simulationLoop();

  RAIINFO(
      std::endl << "-----------------------" << std::endl
                << "Contacts : " << sim->getWorldNumContacts() << std::endl
                << "======================="
  )

  delete sim;
  return 0;
}