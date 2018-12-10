//
// Created by kangd on 18.02.18.
//

#include <boost/program_options.hpp>

#include "BtMbSim.hpp"
#include "raiCommon/utils/StopWatch.hpp"

//#define VIDEO_SAVE_MODE

namespace po = boost::program_options;
std::vector<double> kenergy;
std::vector<double> penergy;

void showplot(double);

int main(int argc, const char* argv[]) {

  bullet_mb_sim::BtMbSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0.8);

  auto ball = sim.addSphere(0.5, 10);
  ball->setPosition(0, 0, 10);

  double g = -0.5;
  double dt = 0.005;
  sim.setGravity({0, 0, g});
  sim.setTimeStep(dt);

  sim.cameraFollowObject(checkerboard, {15.0, 0.0, 15.0});

  double E0 = ball->getEnergy({0, 0, g});
  for(int i = 0; i < int(5.0/dt) && sim.visualizerLoop(0.005, 1.0); i++) {
    sim.integrate();
    kenergy.push_back(ball->getKineticEnergy());
    penergy.push_back(ball->getPotentialEnergy({0, 0, g}));
  }

  showplot(E0);
  return 0;
}

void showplot(double E0) {

  int n = kenergy.size();
  Eigen::MatrixXd tdata(n, 1);
  Eigen::MatrixXd Edata(n, 1);
  Eigen::MatrixXd edata(n, 1);
  Eigen::MatrixXd kEdata(n, 1);
  Eigen::MatrixXd pEdata(n, 1);

  for(int i = 0; i < n; i++) {
    tdata(i, 0) = i;
    Edata(i, 0) = kenergy[i] + penergy[i];
    kEdata(i, 0) = kenergy[i];
    pEdata(i, 0) = penergy[i];
    edata(i, 0) = E0 - (kenergy[i] + penergy[i]);
  }

  rai::Utils::Graph::FigProp2D figure1properties("step", "Energy", "Energy");
  rai::Utils::graph->figure(1, figure1properties);
  rai::Utils::graph->appendData(1,
                                tdata.data(),
                                Edata.data(),
                                n,
                                "E");
  rai::Utils::graph->appendData(1,
                                tdata.data(),
                                kEdata.data(),
                                n,
                                "kE");
  rai::Utils::graph->appendData(1,
                                tdata.data(),
                                pEdata.data(),
                                n,
                                "pE");
  rai::Utils::graph->drawFigure(1);

  rai::Utils::Graph::FigProp2D figure2properties("step", "Energy error", "Energy error");
  rai::Utils::graph->figure(2, figure2properties);
  rai::Utils::graph->appendData(2,
                                tdata.data(),
                                edata.data(),
                                n,
                                "error");
  rai::Utils::graph->drawFigure(2);
  rai::Utils::graph->waitForEnter();
}
