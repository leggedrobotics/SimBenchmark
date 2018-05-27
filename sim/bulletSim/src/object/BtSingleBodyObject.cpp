//
// Created by kangd on 10.02.18.
//

#include "BtSingleBodyObject.hpp"

bullet_sim::object::BtSingleBodyObject::BtSingleBodyObject(double mass) : mass_(mass) {
}

bullet_sim::object::BtSingleBodyObject::~BtSingleBodyObject() {
  delete rigidBody_;
  delete motionState_;
  delete collisionShape_;
}


bool bullet_sim::object::BtSingleBodyObject::isVisualizeFramesAndCom() const {
  return visualizeFramesAndCom_;
}

const benchmark::eQuaternion bullet_sim::object::BtSingleBodyObject::getQuaternion() {
  const btQuaternion &quaternion = rigidBody_->getWorldTransform().getRotation();
  quatTemp_ = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  return quatTemp_.e();
}

void bullet_sim::object::BtSingleBodyObject::getQuaternion(benchmark::Vec<4> &quat) {
  const btQuaternion &quaternion = rigidBody_->getWorldTransform().getRotation();
  quat = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

void bullet_sim::object::BtSingleBodyObject::getRotationMatrix(benchmark::Mat<3, 3> &rotation) {
  const btMatrix3x3 &rotMat = rigidBody_->getWorldTransform().getBasis();
  rotation.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
      rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
      rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();
}

const benchmark::eRotationMat bullet_sim::object::BtSingleBodyObject::getRotationMatrix() {
  const btMatrix3x3 &rotMat = rigidBody_->getWorldTransform().getBasis();
  rotMatTemp_.e() << rotMat.getRow(0).x(), rotMat.getRow(0).y(), rotMat.getRow(0).z(),
      rotMat.getRow(1).x(), rotMat.getRow(1).y(), rotMat.getRow(1).z(),
      rotMat.getRow(2).x(), rotMat.getRow(2).y(), rotMat.getRow(2).z();
  return rotMatTemp_.e();
}

const benchmark::eVector3 bullet_sim::object::BtSingleBodyObject::getPosition() {
  const btVector3 &position = rigidBody_->getWorldTransform().getOrigin();
  posTemp_ = {position.x(), position.y(), position.z()};
  return posTemp_.e();
}

const benchmark::eVector3 bullet_sim::object::BtSingleBodyObject::getComPosition() {
  const btVector3 &position = rigidBody_->getWorldTransform().getOrigin();
  posTemp_ = {position.x(), position.y(), position.z()};
  RAIWARN("check if COM = body origin!");
  return posTemp_.e();
}

const benchmark::eVector3 bullet_sim::object::BtSingleBodyObject::getLinearVelocity() {
  const btVector3 &linearVelocity = rigidBody_->getLinearVelocity();
  linVelTemp_ = {linearVelocity.x(), linearVelocity.y(), linearVelocity.z()};
  return linVelTemp_.e();
}

const benchmark::eVector3 bullet_sim::object::BtSingleBodyObject::getAngularVelocity() {
  const btVector3 &angularVelocity = rigidBody_->getAngularVelocity();
  angVelTemp_ = {angularVelocity.x(), angularVelocity.y(), angularVelocity.z()};
  return angVelTemp_.e();
}

void bullet_sim::object::BtSingleBodyObject::getPosition_W(benchmark::Vec<3> &pos_w) {
  const btVector3 &position = rigidBody_->getWorldTransform().getOrigin();
  pos_w = {position.getX(), position.getY(), position.getZ()};
}

void bullet_sim::object::BtSingleBodyObject::setPosition(Eigen::Vector3d originPosition) {
  rigidBody_->getWorldTransform().setOrigin(btVector3(originPosition[0],
                                                      originPosition[1],
                                                      originPosition[2]));
}

void bullet_sim::object::BtSingleBodyObject::setPosition(double x, double y, double z) {
  rigidBody_->getWorldTransform().setOrigin(btVector3(x, y, z));
}

void bullet_sim::object::BtSingleBodyObject::setOrientation(Eigen::Quaterniond quaternion) {
  setOrientation(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}

void bullet_sim::object::BtSingleBodyObject::setOrientation(Eigen::Matrix3d rotationMatrix) {
  Eigen::Quaterniond quaternion(rotationMatrix);
  setOrientation(quaternion);
}

void bullet_sim::object::BtSingleBodyObject::setOrientation(double w, double x, double y, double z) {
  rigidBody_->getWorldTransform().setRotation(btQuaternion(x, y, z, w));
}

void bullet_sim::object::BtSingleBodyObject::setOrientationRandom() {
  Eigen::Vector4d quat(rn_.sampleUniform(), rn_.sampleUniform(), rn_.sampleUniform(), rn_.sampleUniform());
  quat /= quat.norm();
  setOrientation(quat(0), quat(1), quat(2), quat(3));
}

void bullet_sim::object::BtSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Quaterniond quaternion) {
  setPosition(originPosition);
  setOrientation(quaternion);
}

void bullet_sim::object::BtSingleBodyObject::setPose(Eigen::Vector3d originPosition, Eigen::Matrix3d rotationMatrix) {
  setPosition(originPosition);
  setOrientation(rotationMatrix);
}

void bullet_sim::object::BtSingleBodyObject::setVelocity(Eigen::Vector3d linearVelocity,
                                                       Eigen::Vector3d angularVelocity) {
  setVelocity(linearVelocity[0], linearVelocity[1], linearVelocity[2],
              angularVelocity[0], angularVelocity[1], angularVelocity[2]);
}

void bullet_sim::object::BtSingleBodyObject::setVelocity(double dx,
                                                       double dy,
                                                       double dz,
                                                       double wx,
                                                       double wy,
                                                       double wz) {
  rigidBody_->setLinearVelocity(btVector3(dx, dy, dz));
  rigidBody_->setAngularVelocity(btVector3(wx, wy, wz));
}
btRigidBody *bullet_sim::object::BtSingleBodyObject::getRigidBody() const {
  return rigidBody_;
}
void bullet_sim::object::BtSingleBodyObject::setRestitutionCoefficient(double restitution) {
  rigidBody_->setRestitution(restitution);
}

void bullet_sim::object::BtSingleBodyObject::setFrictionCoefficient(double friction) {
  rigidBody_->setFriction(friction);
}

void bullet_sim::object::BtSingleBodyObject::setExternalForce(Eigen::Vector3d force) {
  rigidBody_->applyCentralForce(btVector3(force[0], force[1], force[2]));
}
void bullet_sim::object::BtSingleBodyObject::setExternalTorque(Eigen::Vector3d torque) {
  rigidBody_->applyTorque(btVector3(torque[0], torque[1], torque[2]));
}

double bullet_sim::object::BtSingleBodyObject::getKineticEnergy() {
  getLinearVelocity();
  getAngularVelocity();
  const btVector3 &localInertia = rigidBody_->getLocalInertia();
  benchmark::Mat<3,3> I;
  I.e() << localInertia.x(), 0, 0,
      0, localInertia.y(), 0,
      0, 0, localInertia.z();

  // ang
  double angEnergy = 0;
  benchmark::Mat<3,3> I_w;
  getRotationMatrix();
  benchmark::similarityTransform(rotMatTemp_, I, I_w);
  benchmark::vecTransposeMatVecMul(angVelTemp_, I_w, angEnergy);

  // lin
  double linEnergy = 0;
  benchmark::vecDot(linVelTemp_, linVelTemp_, linEnergy);

  return 0.5 * angEnergy + 0.5 * mass_ * linEnergy;
}

double bullet_sim::object::BtSingleBodyObject::getPotentialEnergy(const benchmark::Vec<3> &gravity) {
  double potential = 0;
  getPosition();
  benchmark::vecDot(posTemp_, gravity, potential);
  return -potential * mass_;
}

double bullet_sim::object::BtSingleBodyObject::getEnergy(const benchmark::Vec<3> &gravity) {
  return getKineticEnergy() + getPotentialEnergy(gravity);
}

const benchmark::eVector3 bullet_sim::object::BtSingleBodyObject::getLinearMomentum() {
  getLinearVelocity();
  benchmark::vecScalarMul(mass_, linVelTemp_, linearMomentum_);
  return linearMomentum_.e();
}
