//
// Created by kangd on 15.02.18.
//

#include <bulletSim/World_RG.hpp>

#include "thousand.hpp"

int main(int argc, char* argv[]) {

  // main arguments
  int n = 10;
  int shape = 0;    // 0: balls, 1: boxes, 2: capsules
  bool show_graph = false;

  double dt = benchmark::dt;

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
  }

  RAIFATAL_IF(n > 10 || n < 1, "n should be natural number less or equal than 10");
  RAIFATAL_IF(shape > 2 || shape < 0,  "shape should be 0 or 1 or 2");

  // logger
  std::string path = benchmark::dataPath + benchmark::parentDir + "bullet/" + solverName;
  std::string name = std::to_string(dt);
  rai::Utils::logger->setLogPath(path);
  rai::Utils::logger->setLogFileName(name);
  rai::Utils::logger->setOptions(rai::Utils::ONEFILE_FOR_ONEDATA);
  rai::Utils::logger->addVariableToLog(1, "error", "penetration error");

  // timer
  std::string timer = name + "timer";
  rai::Utils::timer->setLogPath(path);
  rai::Utils::timer->setLogFileName(timer);

  // collision world
  bullet_sim::World_RG* sim;

  if (benchmark::visualize)
    sim = new bullet_sim::World_RG(800, 600, 0.5, benchmark::NO_BACKGROUND, solverOption);
  else
    sim = new bullet_sim::World_RG(solverOption);

  // objects
  auto checkerboard = sim->addCheckerboard(5.0, 200.0, 200.0, 0.1,  1, -1, benchmark::GRID);
  std::vector<benchmark::SingleBodyHandle> objectPtrList;

  // random number generator
  rai::RandomNumberGenerator<double> rand;
  rand.seed(benchmark::randomSeed);

  int nPerDim = n;

  benchmark::SingleBodyHandle obj(nullptr, {}, {});

  for(int i = 0; i < nPerDim; i++) {
    for(int j = 0; j < nPerDim; j++) {
      for(int k = 0; k < nPerDim; k++) {

        switch(shape) {
          case 0:
            // sphere (aluminum density)
            obj = sim->addSphere(benchmark::ballR, benchmark::ballM);
            break;
          case 1:
            // box (aluminum density)
            obj = sim->addBox(benchmark::boxSize[0], benchmark::boxSize[1], benchmark::boxSize[2], benchmark::boxM);
            break;
          case 2:
            // capsule (aluminum density)
            obj = sim->addCapsule(benchmark::capsuleSize[0], benchmark::capsuleSize[1], benchmark::capsuleM);
            break;
        }

        // set position
        obj->setPosition((double)i * benchmark::gap + rand.sampleUniform01() * benchmark::perturbation,
                         (double)j * benchmark::gap + rand.sampleUniform01() * benchmark::perturbation,
                         (double)k * benchmark::gap + rand.sampleUniform01() * benchmark::perturbation + benchmark::dropHeight);

        obj->setOrientationRandom();

        if (benchmark::visualize) {
          if((i + j + k) % 3 == 0) {
            obj.visual()[0]->setColor({1.0, 0.0, 0.0});
          }
          else if((i + j + k) % 3 == 1) {
            obj.visual()[0]->setColor({0.0, 1.0, 0.0});
          }
          else if((i + j + k) % 3 == 2) {
            obj.visual()[0]->setColor({0.0, 0.0, 1.0});
          }
        }

        objectPtrList.push_back(obj);
      }
    }
  }

  rai::Utils::timer->startTimer("thousand");
  if(benchmark::visualize) {
    sim->cameraFollowObject(objectPtrList.at(nPerDim*nPerDim*nPerDim/2), {0, 30.0, 10.0});
    // simulation loop with visualizer
    for(int i = 0; i < benchmark::simulationTime / dt && sim->visualizerLoop(dt); i++) {
        sim->integrate(dt);
    }
  }
  else {
    // simulate without visualizer
    for(int i = 0; i < benchmark::simulationTime / dt; i++) {
      // simulate one step
      sim->integrate(dt);
    }
  }
  rai::Utils::timer->stopTimer("thousand");
  rai::Utils::timer->dumpToStdOuput();

  // penetration check (sphere)
  if (shape == 0) {
    double error_sum = 0;
    int numObj = objectPtrList.size();
    for(int i = 0; i < numObj; i++) {
      for(int j = i+1; j < numObj; j++) {
        double dist = (objectPtrList[i]->getPosition() - objectPtrList[j]->getPosition()).norm();

        // error between spheres
        if(dist < benchmark::ballR * 2)
          error_sum += (benchmark::ballR * 2 - dist) * (benchmark::ballR * 2 - dist);
      }
      // error sphere ~ ground
      if(objectPtrList[i]->getPosition()[2] < benchmark::ballR) {
        error_sum += (benchmark::ballR - objectPtrList[i]->getPosition()[2]) * (benchmark::ballR - objectPtrList[i]->getPosition()[2]);
      }
    }

    RAIINFO("penetration error = ");
    RAIINFO(error_sum);
    rai::Utils::logger->appendData("error", error_sum);
  }

  if(show_graph) {
    rai::Utils::Graph::FigPropPieChart propChart;
    rai::Utils::graph->drawPieChartWith_RAI_Timer(0, rai::Utils::timer->getTimedItems(), propChart);
    rai::Utils::graph->drawFigure(0);
    rai::Utils::graph->waitForEnter();
  }
}
