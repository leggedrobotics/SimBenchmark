#ifndef RAI_CHOLINV_HPP
#define RAI_CHOLINV_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

namespace rai {

template<typename Derived, int n_>
static inline void cholInv(Eigen::Matrix<Derived, n_, n_> &A, Eigen::Matrix<Derived, n_, n_> &AInv) {
  unsigned short int i, j, k, n = (unsigned short int) (n_);
  Derived sum;
  Derived p[n];
  memcpy(AInv.data(), A.data(), n * n * sizeof(Derived));

  p[0] = std::sqrt(AInv(0, 0));

  for (j = 1; j < n; j++)
    AInv(j, 0) = AInv(0, j) / p[0];

  for (i = 1; i < n; i++) {
    sum = AInv(i, i);
    for (k = i - 1; k >= 1; k--)
      sum -= AInv(i, k) * AInv(i, k);
    sum -= AInv(i, 0) * AInv(i, 0);
    p[i] = std::sqrt(sum);
    for (j = i + 1; j < n; j++) {
      sum = AInv(i, j);
      for (k = i - 1; k >= 1; k--)
        sum -= AInv(i, k) * AInv(j, k);
      sum -= AInv(i, 0) * AInv(j, 0);
      AInv(j, i) = sum / p[i];
    }
  }

  for (i = 0; i < n; i++) {
    AInv(i, i) = 1.0 / p[i];
    for (j = i + 1; j < n; j++) {
      sum = 0.0;
      for (k = i; k < j; k++) {
        sum -= AInv(j, k) * AInv(k, i);
      }
      AInv(j, i) = sum / p[j];
    }
  }

  for (i = 0; i < n; i++) {
    AInv(i, i) *= AInv(i, i);
    for (k = i + 1; k < n; k++)
      AInv(i, i) += AInv(k, i) * AInv(k, i);

    for (j = i + 1; j < n; j++) {
      AInv(i, j) = AInv(j, i) * AInv(j, j);
      for (k = j + 1; k < n; k++)
        AInv(i, j) += AInv(k, i) * AInv(k, j);
    }
  }

  for (i = 1; i < n; i++)
    for (j = 0; j < i; j++)
      AInv(i, j) = AInv(j, i);

}
}

#endif //RAI_CHOLINV_HPP
