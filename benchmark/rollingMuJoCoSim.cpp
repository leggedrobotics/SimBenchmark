//
// Created by kangd on 21.02.18.
//

#include <mujocoSim/World_RG.hpp>

#include "rolling.hpp"

int main(int argc, char* argv[]) {

  double dt = benchmark::dt;

  mujoco_sim::SolverOption solverOption = mujoco_sim::SOLVER_PGS;
  std::string solverName = "pgs";

  if (argc == 2) {
    dt = atof(argv[1]);
    RAIINFO("----------------------")
    RAIINFO("MuJoCoSim")
    RAIINFO("timestep = " << dt);
  } else if (argc = 3) {
    dt = atof(argv[1]);

    if (strcmp(argv[2], "pgs") == 0) {
      solverOption = mujoco_sim::SOLVER_PGS;
      solverName = "pgs";
    } else if (strcmp(argv[2], "cg") == 0) {
      solverOption = mujoco_sim::SOLVER_CG;
      solverName = "cg";
    } else if (strcmp(argv[2], "newton") == 0) {
      solverOption = mujoco_sim::SOLVER_NEWTON;
      solverName = "newton";
    }

    RAIINFO("----------------------")
    RAIINFO("MuJoCoSim")
    RAIINFO("timestep = " << dt);
    RAIINFO("solver   = " << solverName);
  }

  // logger
  std::string path = benchmark::dataPath + benchmark::parentDir + solverName + "/mujoco";
  std::string name = std::to_string(dt);
  rai::Utils::logger->setOptions(rai::Utils::ONEFILE_FOR_ONEDATA);
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->setLogFileName(name);
  rai::Utils::logger->addVariableToLog(3, "vel_ball", "linear velocity of ball");
  rai::Utils::logger->addVariableToLog(3, "pos_ball", "position of ball");

  // timer
  std::string timer = name + "timer";
  rai::Utils::timer->setLogPath(path);
  rai::Utils::timer->setLogFileName(timer);

  // sim
  // load model from file and check for errors
  mujoco_sim::World_RG *sim;

  if(benchmark::visualize)
    sim = new mujoco_sim::World_RG(800, 600, 0.5, "/home/kangd/git/benchmark/benchmark/mujoco/rolling.xml", benchmark::NO_BACKGROUND, solverOption);
  else
    sim = new mujoco_sim::World_RG("/home/kangd/git/benchmark/benchmark/mujoco/rolling.xml", solverOption);


  // simulation loop
  // press 'q' key to quit
  rai::Utils::timer->startTimer("rolling");
  if(benchmark::visualize) {
    // camera relative position
    sim->setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);
    sim->cameraFollowObject(sim->getSingleBodyHandle(0), {30, 0, 10});

    for(int i = 0; i < benchmark::simulationTime / dt && sim->visualizerLoop(dt); i++) {
      sim->getSingleBodyHandle(1)->setExternalForce(benchmark::force);
      // log
      rai::Utils::logger->appendData("linvel_box", sim->getSingleBodyHandle(1)->getLinearVelocity().data());
      rai::Utils::logger->appendData("linvel_ball", sim->getSingleBodyHandle(2)->getLinearVelocity().data());
      rai::Utils::logger->appendData("pos_box", sim->getSingleBodyHandle(1)->getPosition().data());
      rai::Utils::logger->appendData("pos_ball", sim->getSingleBodyHandle(2)->getPosition().data());
      sim->integrate(dt);
    }
  }
  else {
    for(int i = 0; i < benchmark::simulationTime / dt; i++) {
      sim->getSingleBodyHandle(1)->setExternalForce(benchmark::force);
      // log
      rai::Utils::logger->appendData("linvel_box", sim->getSingleBodyHandle(1)->getLinearVelocity().data());
      rai::Utils::logger->appendData("linvel_ball", sim->getSingleBodyHandle(2)->getLinearVelocity().data());
      rai::Utils::logger->appendData("pos_box", sim->getSingleBodyHandle(1)->getPosition().data());
      rai::Utils::logger->appendData("pos_ball", sim->getSingleBodyHandle(2)->getPosition().data());
      sim->integrate(dt);
    }
  }
  rai::Utils::timer->stopTimer("rolling");

  // delete sim
  delete sim;

  // time log
  rai::Utils::timer->dumpToStdOuput();

  return 0;
}
