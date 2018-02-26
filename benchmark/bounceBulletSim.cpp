//
// Created by kangd on 15.02.18.
//

#include <bulletSim/World_RG.hpp>

#include "bounce.hpp"

int main(int argc, char* argv[]) {

  double dt = benchmark::dt;
  double restitution = benchmark::restitution;

  bullet_sim::SolverOption solverOption = bullet_sim::SOLVER_SEQUENTIAL_IMPULSE;
  std::string solverName = "seqImp";

  if (argc == 2) {
    dt = atof(argv[1]);
    RAIINFO("----------------------")
    RAIINFO("BulletSim")
    RAIINFO("timestep = " << dt);
  } else if (argc == 3) {
    dt = atof(argv[1]);

    if(strcmp(argv[2],"seqImp")==0) {
      solverOption = bullet_sim::SOLVER_SEQUENTIAL_IMPULSE;
      solverName = "seqImp";
    } else if(strcmp(argv[2],"nncg")==0) {
      solverOption = bullet_sim::SOLVER_NNCG;
      solverName = "nncg";
    } else if(strcmp(argv[2],"pgs")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_PGS;
      solverName = "pgs";
    } else if(strcmp(argv[2],"dantzig")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_DANTZIG;
      solverName = "dantzig";
    } else if(strcmp(argv[2],"lemke")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_LEMKE;
      solverName = "lemke";
    }

    RAIINFO("----------------------")
    RAIINFO("BulletSim")
    RAIINFO("timestep = " << dt);
    RAIINFO("solver   = " << solverName);
  } else if (argc == 4) {
    dt = atof(argv[1]);

    if(strcmp(argv[2],"seqImp")==0) {
      solverOption = bullet_sim::SOLVER_SEQUENTIAL_IMPULSE;
      solverName = "seqImp";
    } else if(strcmp(argv[2],"nncg")==0) {
      solverOption = bullet_sim::SOLVER_NNCG;
      solverName = "nncg";
    } else if(strcmp(argv[2],"pgs")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_PGS;
      solverName = "pgs";
    } else if(strcmp(argv[2],"dantzig")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_DANTZIG;
      solverName = "dantzig";
    } else if(strcmp(argv[2],"lemke")==0) {
      solverOption = bullet_sim::SOLVER_MLCP_LEMKE;
      solverName = "lemke";
    }

    restitution = atof(argv[3]);

    RAIINFO("----------------------")
    RAIINFO("BulletSim")
    RAIINFO("timestep    = " << dt);
    RAIINFO("solver      = " << solverName);
    RAIINFO("restitution = " << restitution);
  }

  // logger
  std::string path = benchmark::dataPath + benchmark::parentDir + "bullet/" + solverName;
  std::string name = std::to_string(dt);
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->setLogFileName(name);
  rai::Utils::logger->setOptions(rai::Utils::ONEFILE_FOR_ONEDATA);
  rai::Utils::logger->addVariableToLog(3, "velball", "linear velocity of ball");
  rai::Utils::logger->addVariableToLog(3, "posball", "position of ball");

  // timer
  std::string timer = name + "timer";
  rai::Utils::timer->setLogPath(path);
  rai::Utils::timer->setLogFileName(timer);

  // sim
  bullet_sim::World_RG *sim;

  if(benchmark::visualize)
    sim = new bullet_sim::World_RG(800, 600, 0.5, benchmark::NO_BACKGROUND, solverOption);
  else
    sim = new bullet_sim::World_RG(solverOption);

  sim->setGravity(benchmark::gravity);
  sim->setERP(benchmark::erp, benchmark::erp, benchmark::erp);

  // add objects
  // checkerboard
  auto checkerboard = sim->addCheckerboard(5.0, 100.0, 100.0, 0.1);
  checkerboard->setRestitutionCoefficient(restitution);
  checkerboard->setFrictionCoefficient(benchmark::friction);

  std::vector<benchmark::SingleBodyHandle> objectList;

  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      auto ball = sim->addSphere(0.5, 1);
      ball->setPosition(i * 2.0 - 4.0, j * 2.0 - 4.0, benchmark::dropHeight);
      ball->setRestitutionCoefficient(restitution);
      ball->setFrictionCoefficient(benchmark::friction);
      objectList.push_back(ball);
    }
  }

  // simulation loop
  // press 'q' key to quit
  rai::Utils::timer->startTimer("bounce");
  if(benchmark::visualize) {
    // camera relative position
    sim->setLightPosition(benchmark::lightX, benchmark::lightY, benchmark::lightZ);
    sim->cameraFollowObject(objectList[12], {10, 0, 5});

    for(int i = 0; i < benchmark::simulationTime / dt && sim->visualizerLoop(dt); i++) {
      // log
      rai::Utils::logger->appendData("velball", objectList[12]->getLinearVelocity().data());
      rai::Utils::logger->appendData("posball", objectList[12]->getPosition().data());
      sim->integrate(benchmark::dt);
    }
  }
  else {
    for(int i = 0; i < benchmark::simulationTime / dt && sim->visualizerLoop(dt); i++) {
      // log
      rai::Utils::logger->appendData("velball", objectList[12]->getLinearVelocity().data());
      rai::Utils::logger->appendData("posball", objectList[12]->getPosition().data());
      sim->integrate(benchmark::dt);
    }
  }
  rai::Utils::timer->stopTimer("bounce");

  // delete sim
  delete sim;

  // time log
  rai::Utils::timer->dumpToStdOuput();

  return 0;
}