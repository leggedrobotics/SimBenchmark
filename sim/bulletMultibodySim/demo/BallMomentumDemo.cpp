//
// Created by kangd on 18.02.18.
//

#include <boost/program_options.hpp>

#include "BtMbSim.hpp"
#include "raiCommon/utils/StopWatch.hpp"

//#define VIDEO_SAVE_MODE

namespace po = boost::program_options;
std::vector<Eigen::Vector3d> kenergy;

void showplot();

int main(int argc, const char* argv[]) {

  bullet_mb_sim::BtMbSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);
  sim.setGravity({0, 0, 0});
  sim.setTimeStep(0.005);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0.8);

  auto ball1 = sim.addSphere(0.1, 10);
  ball1->setPosition(0, -5, 2);
  ball1->setVelocity(0, 2, 0, 0, 0, 0);
  auto ball2 = sim.addSphere(0.1, 10);
  ball2->setPosition(0, 0, 2);

  sim.cameraFollowObject(checkerboard, {5.0, 0.0, 5.0});

  for(int i = 0; i < 1000 && sim.visualizerLoop(0.005, 1.0); i++) {
    sim.integrate();
    Eigen::Vector3d mo = ball1->getLinearMomentum() + ball2->getLinearMomentum();
    kenergy.push_back(mo);
  }

  showplot();
  return 0;
}

void showplot() {

  int n = kenergy.size();
  Eigen::MatrixXd tdata(n, 1);
  Eigen::MatrixXd xdata(n, 1);
  Eigen::MatrixXd ydata(n, 1);
  Eigen::MatrixXd zdata(n, 1);

  for(int i = 0; i < n; i++) {
    Eigen::Vector3d data = kenergy[i];
    tdata(i, 0) = i;
    xdata(i, 0) = data.x();
    ydata(i, 0) = data.y();
    zdata(i, 0) = data.z();
  }

  rai::Utils::Graph::FigProp2D figure1properties("step", "momentum", "momentum");
  rai::Utils::graph->figure(1, figure1properties);
//  rai::Utils::graph->appendData(1,
//                                tdata.data(),
//                                xdata.data(),
//                                n,
//                                "x");
  rai::Utils::graph->appendData(1,
                                tdata.data(),
                                ydata.data(),
                                n,
                                "y");
//  rai::Utils::graph->appendData(1,
//                                tdata.data(),
//                                zdata.data(),
//                                n,
//                                "z");
  rai::Utils::graph->drawFigure(1);
  rai::Utils::graph->waitForEnter();
}
