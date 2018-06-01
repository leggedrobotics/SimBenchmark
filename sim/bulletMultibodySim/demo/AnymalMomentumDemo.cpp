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

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/ANYmal/robot.urdf";

  bullet_mb_sim::BtMbSim sim(800, 600, 0.5, benchmark::NO_BACKGROUND);

  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, bo::PLANE_SHAPE, 1, -1, bo::GRID);
  checkerboard->setFrictionCoefficient(0.8);

  auto anymal = sim.addArticulatedSystem(urdfPath, bullet_mb_sim::object::URDF, false);
  anymal->setGeneralizedCoordinate(
      {0, 0, 3,
       1.0, 0.0, 0.0, 0.0,
       0.03, 0.4, -0.8,
       -0.03, 0.4, -0.8,
       0.03, -0.4, 0.8,
       -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity(
      {0, 1, 0,
       0.2, 1, 0.5,
       0.1, 0.2, 0.3,
       0.4, 0.5, 0.6,
       0.7, 0.8, 0.9,
       1.0, 1.1, 1.2});

  sim.setGravity({0, 0, 0});
  sim.setTimeStep(0.005);

  sim.cameraFollowObject(checkerboard, {5.0, 0.0, 5.0});

  for(int i = 0; i < 1000 && sim.visualizerLoop(0.005, 1.0); i++) {
    sim.integrate();
    kenergy.push_back(anymal->getLinearMomentumInCartesianSpace());
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
  rai::Utils::graph->appendData(1,
                                tdata.data(),
                                xdata.data(),
                                n,
                                "x");
  rai::Utils::graph->appendData(1,
                                tdata.data(),
                                ydata.data(),
                                n,
                                "y");
  rai::Utils::graph->appendData(1,
                                tdata.data(),
                                zdata.data(),
                                n,
                                "z");
  rai::Utils::graph->drawFigure(1);
  rai::Utils::graph->waitForEnter();
}
