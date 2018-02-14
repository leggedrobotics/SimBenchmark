//
// Created by kangd on 13.02.18.
//

#ifndef BULLETSIM_ARTICULATEDSYSTEM_HPP
#define BULLETSIM_ARTICULATEDSYSTEM_HPP

#include <iostream>
#include <fstream>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Core>

class ArticulatedSystem {


 public:

  typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;
  typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

  explicit ArticulatedSystem(std::string urdfFile, std::vector<std::string> jointOrder = std::vector<std::string>());

  const EigenVec getGeneralizedCoordinate();

  const EigenVec getGeneralizedVelocity();

  /* For floating-Interface robots, [linearPosition_W, baseRationInQuaternion, joint Angles]
   * For fixed-Interface robot, [joint angles]
   * The dimension is the DOF+1 for floating-based, and DOF for fixed based. (obtained by getDOF())*/
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState);

  /* For floating-Interface robots, [linearVelocity_W, angularVelocity_W, jointVelocity]
   * The dimension is the same as dof (obtained with getDOF)*/
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel);

  void setGeneralizedCoordinate(std::initializer_list<double> jointState);

  void setGeneralizedVelocity(std::initializer_list<double> jointVel);

  void setGeneralizedForce(std::initializer_list<double> tau);

  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel);

  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel);

  void setGeneralizedForce(const Eigen::VectorXd &tau);

  const EigenVec getGeneralizedForce();

  EigenMat getMassMatrix() {return M_.e();}

  EigenVec getNonlinearities() {return h_.e();}

  EigenMat getInverseMassMatrix() {return Minv_.e();}

  void printOutBodyNamesInOrder();

  /* frames are attached to every joint coordinate */
  void printOutFrameNamesInOrder();

  /* returns position in the world frame of a point defined in a joint frame*/
  void getPosition_W(int bodyIdx, const Vec<3> &point_B, Vec<3> &point_W);

  /* returns position in the world frame of a frame defined in the robot description*/
  void getPosition_W(int frameId, Vec<3> &point_W);

  void getOrientationW2B(int frameId, Mat<3, 3> &orientation_W);

  void getSparseJacobian(int bodyIdx, const Vec<3> &point_W, SparseJacobian &jaco);

  void getSparseJacobianTimeDerivative(int bodyIdx, const Vec<3> &point_W, SparseJacobian &Dt_jaco);

  void getSparseRotationalJacobianTimeDerivative(int bodyIdx, const Vec<3> &point_W, SparseJacobian &Dt_jaco);

  void getSparseRotationalJacobian(int bodyIdx, const Vec<3> &point_W, SparseJacobian &jaco);

  /* This method assumes that you already resized the eigen Matrix appropriately (3 by dof) and
   * the empty columns are already set to zero. */
  void convertSparseJacobianToDense(SparseJacobian &sparseJaco, Eigen::MatrixXd& denseJaco);

  /* This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization */
  void getDenseJacobian(int bodyIdx, const Vec<3> &point_W, Eigen::MatrixXd &jaco);

  /* This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization */
  void getDenseRotationalJacobian(int bodyIdx, Eigen::MatrixXd &jaco);

  void getVelocity_W(const SparseJacobian &jaco, Vec<3> &pointVel);

  void getVelocity_W(int bodyId, const Vec<3> &posInBodyFrame, Vec<3> &pointVel);

  void getAngularVelocity_W(int bodyId, Vec<3> &angVel);

  int getFrameIdx(std::string nm);

  int getBodyIdx(std::string nm);

  CoordinateFrame &getFrame(std::string nm);

  CoordinateFrame &getFrame(int idx);

  void preContactSolverUpdate1(const Vec<3> &gravity, double dt);

  int getDOF();

  /// returns the dim of generalized coordinate
  int getStateDim();

  double getEnergy(const Vec<3> &gravity);

  void getBodyPose(int bodyId, Mat<3, 3> &orientation, Vec<3> &position);

  std::vector<rai_sim::Vec<3>> &getJointPos_P();

  double getMass(int localIdx);

#ifndef RAI_BUILD_AS_ONLY
  void setJointDamping(Eigen::VectorXd dampingCoefficient);
#endif

  void setExternalForce(Vec<3>& force, int localIdx);

  void setExternalTorque(Vec<3>& torque, int localIdx);

 protected:
  CollisionSet &getCollisionObj();

  void updateCollision();

  ObjectType getObjectType();

  bool isMovable(int localIdx) const;

  void updateKinematics();

  void computeMassMatrix(MatDyn &M);

  void computeNonlinearities(const Vec<3> &gravity, VecDyn &b);

  void computeSparseInverse(const MatDyn &M, MatDyn &Minv);

#ifndef RAI_BUILD_AS_ONLY
  void destroyCollisionBodies();

  /// computing JM^-1J^T exploiting sparsity
  void getFullDelassusAndTauStar(const MatDyn &Minv,
                                 const std::vector<SparseJacobian> &jaco,
                                 double dt,
                                 const VecDyn &h,
                                 std::vector<std::pair<std::vector<Mat<3, 3>>, Vec<3>>> &delassusAndTauStar);

  void preContactSolverUpdate2(const Vec<3> &gravity, double dt);

  void integrate(double dt, double erp);

  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape, rai_sim::Vec<4>>>& getVisOb() {
    return visObj;
  };

  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape>>& getVisColOb() {
    return visColObj;
  };

#endif

 private:

  void init();

  void computeExpandedParentArray();

  bool processLinkFromUrdf(boost::shared_ptr<const urdf::Link> urdfLink, Child &raiLink, std::vector<std::string> &jointsOrder);

  /// for computation
  /// Frames:: W: world, B: body, P: parent

  /// changing variables
  std::vector<rai_sim::Mat<3, 3>> rot_WB;
  std::vector<rai_sim::Mat<3, 3>> rot_PB;
  std::vector<rai_sim::Vec<3>> jointAxis_W;
  std::vector<rai_sim::Vec<3>> jointPos_W;
  std::vector<rai_sim::Vec<3>> joint2com_W;
  std::vector<rai_sim::Vec<3>> comPos_W;
  std::vector<rai_sim::Mat<3, 3>> compositeInertia_W;
  std::vector<rai_sim::Vec<3>> composite_mXCOM_W;
  std::vector<rai_sim::Vec<3>> composite_com_W;
  std::vector<rai_sim::Vec<3>> bodyLinearVel_W;
  std::vector<rai_sim::Vec<3>> bodyAngVel_W;
  std::vector<rai_sim::Vec<3>> bodyAngVel_pc_W;
  std::vector<rai_sim::Vec<3>> bodyLinVel_pc_W;
  std::vector<rai_sim::Vec<3>> bodyLinearAcc;
  std::vector<rai_sim::Vec<3>> bodyAngAcc;
  std::vector<rai_sim::Vec<3>> propagatingForceAtJoint_W;
  std::vector<rai_sim::Vec<3>> propagatingTorqueAtJoint_W;
  std::vector<rai_sim::Mat<3, 3>> inertiaAboutJoint_W;
  std::vector<rai_sim::Mat<3, 3>> inertia_comW;
  std::vector<rai_sim::Vec<3>> sketch;
  std::vector<rai_sim::Vec<3>> joint2joint_W;
  std::vector<Joint::Type> jointType;

  /// constant params
  std::vector<rai_sim::Mat<3, 3>> rot_JB;
  std::vector<rai_sim::Vec<3>> jointPos_P;
  std::vector<rai_sim::Vec<3>> jointAxis_P;
  std::vector<rai_sim::Mat<3, 3>> inertia_comB;
  std::vector<rai_sim::Vec<3>> comPos_B;

  std::vector<double> mass;
  std::vector<int> parent;
  std::vector<int> leafnodes_;
  std::vector<double> compositeMass;

  std::vector<std::vector<int> > children_;
  std::vector<int> toBaseCount_; /// counts to base (which is WORLD).
  std::vector<int> lambda;

  VecDyn jointState_, jointVel_, h_, dampingCoeff_, jointVelERP_;
  MatDyn M_, Minv_, MinvJ2_;
  VecDyn tauStar_, tau_;

  std::vector<CoordinateFrame> frameOfInterest_;
  std::vector<std::string> jointsNames_;

  Joint::Type baseType_;

 protected:
  std::vector<Child> child_;
  std::vector<std::string> bodyName;
  std::vector<SparseJacobian> contactJaco_;
  std::vector<SparseJacobian> externalForceJaco_;
  std::vector<rai_sim::Vec<3>> externalForces_;

#ifndef RAI_BUILD_AS_ONLY
 protected:
  rai_sim::CollisionSet collisionBodies;
  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape>> visColObj;
  std::vector<std::tuple<rai_sim::Mat<3, 3>, rai_sim::Vec<3>, int, Shape, rai_sim::Vec<4>>> visObj;
  std::vector<std::pair<std::string, rai_sim::Vec<4>>> visProps_;
  std::vector<std::pair<std::string, rai_sim::Vec<4>>> visColProps_;
#endif

 private:
  int nbody, dof = -1, jointStateDim = 0, baseJointStateDimMinusOne = 0, baseDOFminusOne = 0;

};

#endif //BULLETSIM_ARTICULATEDSYSTEM_HPP
