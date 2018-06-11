//
// Created by jhwangbo on 30.11.16.
//

#ifndef RAI_TYPEDEF_HPP
#define RAI_TYPEDEF_HPP

#include <Eigen/Core>

namespace rai {

typedef typename Eigen::Matrix<double, 4, 1> Quaternion;
typedef typename Eigen::Matrix<double, 3, 1> EulerVector;
typedef typename Eigen::Matrix<double, 3, 3> RotationMatrix;
typedef typename Eigen::Matrix<double, 4, 4> HomogeneousTransform;
typedef typename Eigen::Matrix<double, 3, 1> Position;
typedef typename Eigen::Matrix<double, 3, 1> AngularVelocity;
typedef typename Eigen::Matrix<double, 3, 1> AngularAcceleration;
typedef typename Eigen::Matrix<double, 3, 1> LinearVelocity;
typedef typename Eigen::Matrix<double, 3, 1> LinearAcceleration;
typedef typename Eigen::Matrix<double, 3, 1> Axis;
typedef typename Eigen::Matrix<double, 3, 1> Torque;
typedef typename Eigen::Matrix<double, 3, 1> Force;
typedef typename Eigen::Matrix<double, 3, 3> Inertia;

typedef typename Eigen::Matrix<float, 4, 1> Quaternionf;
typedef typename Eigen::Matrix<float, 3, 1> EulerVectorf;
typedef typename Eigen::Matrix<float, 3, 3> RotationMatrixf;
typedef typename Eigen::Matrix<float, 3, 1> Positionf;
typedef typename Eigen::Matrix<float, 3, 1> AngularVelocityf;
typedef typename Eigen::Matrix<float, 3, 1> AngularAccelerationf;
typedef typename Eigen::Matrix<float, 3, 1> LinearVelocityf;
typedef typename Eigen::Matrix<float, 3, 1> LinearAccelerationf;
typedef typename Eigen::Matrix<float, 3, 1> Axisf;
typedef typename Eigen::Matrix<float, 3, 1> Torquef;
typedef typename Eigen::Matrix<float, 3, 1> Forcef;
typedef typename Eigen::Matrix<float, 3, 3> Inertiaf;

typedef typename Eigen::MatrixXd MatrixXd;
typedef typename Eigen::VectorXd VectorXd;

}

#endif //RAI_TYPEDEF_HPP
