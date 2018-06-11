//
// Created by jhwangbo on 23.09.16.
// This function is to solve linear equation Ax=b

#ifndef RAI_CONJUGATEGRADIENT_HPP
#define RAI_CONJUGATEGRADIENT_HPP
#include <functional>
#include <Eigen/Core>
#include <iostream>

namespace rai{
//namespace Math{

template <typename Derived>
Derived conjugateGradient(std::function< void(Eigen::Matrix<Derived, -1, 1>&,
                                           Eigen::Matrix<Derived, -1, 1>&)> eval,
                       Eigen::Matrix<Derived, -1, 1> b,
                       int iterN, Derived tol, Eigen::Matrix<Derived, -1, 1>& sol)
{
  int dim = b.rows();
  Eigen::Matrix<Derived, -1, 1> x, p, r, Ap, Ar, pnew, Apnew;
  x.setZero(dim); Ap.setZero(dim); Ar.setZero(dim); pnew.setZero(dim);
  r = b;
  p = b;

  Derived rdotr = r.squaredNorm();
  Derived newrdotr=0;
  int i;
  for(i = 0; i < iterN && i < dim; i++){
    eval(p, Ap);
    Derived a = rdotr / p.dot(Ap);
    x += a * p;
    r -= a * Ap;
    newrdotr = r.dot(r);
    if (newrdotr < tol)
      break;
    Derived b = newrdotr/rdotr;
    p = r + b * p;
    rdotr = newrdotr;
  }
  sol = x;
  return newrdotr;
};

}
//}

#endif //RAI_CONJUGATEGRADIENT_HPP
