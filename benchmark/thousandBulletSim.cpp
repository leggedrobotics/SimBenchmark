//
// Created by kangd on 15.02.18.
//

#include <bulletSim/World_RG.hpp>

int main(int argc, char* argv[]) {

  // main arguments
  int n = 10;
  int shape = 0;    // 0: balls, 1: boxes, 2: capsules
  bool is_visualization_mode = true;
  bool show_graph = true;

  if (argc == 5) {
    n = atoi(argv[1]);                            // n
    shape = atoi(argv[2]);                        // shape
    is_visualization_mode = (atoi(argv[3]) != 0); // visulization on/off
    show_graph = (atoi(argv[4]) != 0);            // graph on/off
  }

  RAIFATAL_IF(n > 10 || n < 1, "n should be natural number less or equal than 10");
  RAIFATAL_IF(shape > 2 || shape < 0,  "shape should be 0 or 1 or 2");

  // print benchmark parameters
  RAIINFO(  std::endl
                << "===========================================" << std::endl
                << "Benchmark parameters: " << std::endl
                << "shape       = " << shape << "   (0: balls / 1: boxes / 2 : capsules)" << std::endl
                << "n           = " << n << "   (total n^3 objects)" << std::endl
                << "visualizer  = " << is_visualization_mode << "   (0: false / 1: true)" << std::endl
                << "===========================================");

  // timer
  std::string path = "/tmp";
  rai::Utils::timer->setLogPath(path);
  rai::Utils::logger->setLogPath(path);

  // collision world
  bullet_sim::World_RG* sim;

  if (is_visualization_mode)
    sim = new bullet_sim::World_RG(800, 600, 0.6);

  // objects
  auto checkerboard = sim->addCheckerboard(5.0, 200.0, 200.0, 0.1);
  std::vector<bullet_sim::SingleBodyHandle> objectPtrList;

  // random number generator
  rai::RandomNumberGenerator<double> rand;

  int nPerDim = n;
  const double perturbation = 0.001;
  const double dropHeight = 5.0;

  bullet_sim::SingleBodyHandle obj(nullptr, {}, {});

  for(int i = 0; i < nPerDim; i++) {
    for(int j = 0; j < nPerDim; j++) {
      for(int k = 0; k < nPerDim; k++) {

        switch(shape) {
          case 0:
            // sphere (aluminum density)
            obj = sim->addSphere(0.5, 1500.0);
            break;
          case 1:
            // box (aluminum density)
            obj = sim->addBox(0.8, 0.4, 0.2, 170.0);
            break;
          case 2:
            // capsule (aluminum density)
            obj = sim->addCapsule(0.2, 0.6, 300.0);
            break;
        }

        // set position
        obj->setPosition((double)i * 1.1 + rand.sampleUniform01() * perturbation,
                         (double)j * 1.1 + rand.sampleUniform01() * perturbation,
                         (double)k * 1.1 + rand.sampleUniform01() * perturbation + dropHeight);

        obj->setOrientationRandom();

        if (is_visualization_mode) {
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

  double dt = 0.01;  // (sec)

  rai::Utils::timer->startTimer("10000IterationBenchmark");
  if(is_visualization_mode) {
    sim->cameraFollowObject(objectPtrList.at(nPerDim*nPerDim*nPerDim/2), {0, 30.0, 10.0});
    // simulation loop with visualizer
    sim->loop(dt);
  }
  else {
    // simulate 10000 iteration without visualizer
    for(int i = 0; i < 1000; i++) {
      // simulate one step
      sim->integrate(dt);
    }
  }
  rai::Utils::timer->stopTimer("10000IterationBenchmark");
  rai::Utils::timer->dumpToStdOuput();

  if(show_graph) {
    rai::Utils::Graph::FigPropPieChart propChart;
    rai::Utils::graph->drawPieChartWith_RAI_Timer(0, rai::Utils::timer->getTimedItems(), propChart);
    rai::Utils::graph->drawFigure(0);
    rai::Utils::graph->waitForEnter();
  }
}
