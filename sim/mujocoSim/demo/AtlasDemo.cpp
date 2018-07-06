//
// Created by jhwangbo on 04/12/17.
//
#include <MjcSim.hpp>

//#define GUI

int main() {

  std::string urdfPath(__FILE__);
  while (urdfPath.back() != '/')
    urdfPath.erase(urdfPath.size() - 1, 1);
  urdfPath += "../../../res/demo/mujoco/Atlas/robot.urdf";

  std::string keyPath(__FILE__);
  while (keyPath.back() != '/')
    keyPath.erase(keyPath.size() - 1, 1);
  keyPath += "../../../lib/mjpro150/mjkey.txt";

#ifdef GUI
  mujoco_sim::MjcSim sim(800, 600, 0.2,
                         urdfPath.c_str(),
                         keyPath.c_str(),
                         benchmark::NO_BACKGROUND,
                         mujoco_sim::SOLVER_CG);
#else
  mujoco_sim::MjcSim sim(urdfPath.c_str(),
                         keyPath.c_str(),
                         mujoco_sim::SOLVER_CG);
#endif

  Eigen::VectorXd gc(sim.getStateDimension());
  Eigen::VectorXd gv(sim.getDOF()), tau(sim.getDOF());

  std::cout<<"dof "<<sim.getDOF()<<"\n";

  gc.setZero();
//  gc.segment<7>(0) << 0,0,3,1,0,0,0;
  gc.segment<7>(0) << 0,0,0.3,0.7071,0,0.7071,0;
  gv.setZero();
  tau.setZero();

  sim.setState(gc, gv);

  const double dt = 0.005;
  int counter = 0;
  sim.setTimeStep(dt);

  StopWatch watch;
  watch.start();

#ifdef GUI
  sim.setLightPosition(0, -10, 10);
  sim.cameraFollowObject(sim.getSingleBodyHandle(0), {5, 0, 5});
  for(int i=0; i<100000 && sim.visualizerLoop(dt); i++)
#else
  for(int i=0; i<100000; i++)
#endif
  {
    sim.integrate();
    counter += sim.getWorldNumContacts();
  }

  std::cout << "Number of steps in one second  "<< 100.0 / watch.measure() << "k\n";
  std::cout << "contact problem size: "<< float(counter) / 100000 <<"\n";
}