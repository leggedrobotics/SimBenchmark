//
// Created by kangd on 13.02.18.
//

#include <raiSim/World_RG.hpp>

//#define SIM_TIME_MODE
//#define VIDEO_SAVE_MODE

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/ANYmal/";

#if defined(SIM_TIME_MODE)
  rai_sim::World_RG sim;
#else
  rai_sim::World_RG sim(800, 600, 0.5, rai_sim::NO_BACKGROUND);
#endif
  sim.setGravity({0, 0, -9.8});

  // add objects
  auto checkerboard = sim.addCheckerboard(2, 100, 100, 0.1, 1, -1, rai_sim::GRID);

  Eigen::VectorXd jointNominalConfig(19);
  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);

  jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  std::vector<rai_sim::ArticulatedSystemHandle> animals;

  auto anymal = sim.addArticulatedSystem(urdfPath);
  anymal->setGeneralizedCoordinate({0, 0, 0.54,
                                    1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
                                    -0.8, -0.03, 0.4, -0.8, 0.03, -0.4,
                                    0.8, -0.03, -0.4, 0.8});
  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  // set color
  for(int i = 0; i < anymal.visual().size(); i++) {
    anymal.visual()[i]->setColor({0, 1, 0});
  }

  const double kp = 40.0, kd = 1.0;

#if defined(SIM_TIME_MODE)
  StopWatch watch;
  watch.start();
  for(int i = 0; i < 50000; i++) {
#else
  sim.cameraFollowObject(checkerboard, {1.0, 1.0, 1.0});
#if defined(VIDEO_SAVE_MODE)
  sim.startRecordingVideo("/tmp", "raiAnymal");
  for(int i = 0; i < 2000 && sim.visualizerLoop(0.005, 1.0); i++) {
#else
    while(sim.visualizerLoop(0.005, 1.0)) {
#endif
#endif
    sim.integrate1(0.005);

    jointState = anymal->getGeneralizedCoordinate();
    jointVel = anymal->getGeneralizedVelocity();
    jointForce = anymal->getGeneralizedForce();

    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
    jointForce.head(6).setZero();
    anymal->setGeneralizedForce(jointForce);

    sim.integrate2(0.005);
  }

#if defined(SIM_TIME_MODE)
  std::cout<<"time taken for 50k steps "<< watch.measure()<<"s \n";
#elif defined(VIDEO_SAVE_MODE)
  sim.stopRecordingVideo();
#endif

  return 0;
}