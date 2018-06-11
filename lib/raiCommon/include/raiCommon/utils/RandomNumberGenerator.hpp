/*
 * randomNumberGenerator.hpp
 *
 *  Created on: Apr 29, 2016
 *      Author: jhwangbo
 */

#ifndef RANDOMNUMBERGENERATOR_HPP_
#define RANDOMNUMBERGENERATOR_HPP_

// for random sampling
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/math/distributions.hpp>
#include <cstdlib>
#include <mutex>
#include <Eigen/Core>

namespace rai {

template<typename Dtype>
class RandomNumberGenerator {

 public:

  RandomNumberGenerator() {
//    distributionNormal = new boost::random::normal_distribution<Dtype>(0.0, 1.0);
//    distributionUni = new boost::uniform_real<Dtype>(-1, 1);
//    distributionUni01 = new boost::uniform_real<Dtype>(0, 1);
  }

  ~RandomNumberGenerator() {
//    delete distributionNormal;
//    delete distributionUni;
//    delete distributionUni01;
  }

  /* mean =0, std = 1*/
  Dtype sampleNormal() {
    std::lock_guard<std::mutex> lockModel(rndMutex_);
    auto dist = boost::random::normal_distribution<Dtype>(0.0, 1.0);
    return dist(rngGenerator);
  }

  /* from -1 to 1*/
  Dtype sampleUniform() {
    auto dist = boost::uniform_real<Dtype>(-1, 1);
    std::lock_guard<std::mutex> lockModel(rndMutex_);
    return dist(rngGenerator);
  }

  /* from 0 to 1*/
  Dtype sampleUniform01() {
    auto dist = boost::uniform_real<Dtype>(0, 1);
    std::lock_guard<std::mutex> lockModel(rndMutex_);
    return dist(rngGenerator);
  }

  bool forXPercent(float epsilon) {
    auto dist = boost::uniform_real<Dtype>(0, 1);
    std::lock_guard<std::mutex> lockModel(rndMutex_);
    return dist(rngGenerator) < epsilon;
  }

  int intRand(const int &min, const int &max) {
    {
      std::lock_guard<std::mutex> lockModel(rndMutex_);
      std::uniform_int_distribution<int> distribution(min, max);
      return distribution(rngGenerator);
    }
  }

  /* weighted random sampling where the weights are given by 1, r, r^2, ... */
  int intWeightedRand (const int &max, Dtype weightDecayFtr) {
    Dtype sum = (Dtype(1) - std::pow(weightDecayFtr, max+1)) / (Dtype(1)-weightDecayFtr);
    return std::ceil(log(Dtype(1) - sampleUniform01() * sum * (Dtype(1) - weightDecayFtr)) / log(weightDecayFtr)) - 1;
  }

  template<int dim>
  void sampleVectorInNormalUniform(Dtype *vector) {
    std::lock_guard<std::mutex> lockModel(rndMutex_);
    auto dist = boost::uniform_real<Dtype>(-1, 1);
    for (int i = 0; i < dim; i++)
      vector[i] = dist(rngGenerator);
  }

  template<int dim>
  void sampleInUnitSphere(Dtype *vector) {
    sampleVectorInNormalUniform<dim>(vector);
    Dtype sum = 0.0f;

    for (int i = 0; i < dim; i++)
      sum += vector[i] * vector[i];

    Dtype amplitudeOverSum = pow(std::abs(sampleUniform()), Dtype(1.0) / Dtype(dim)) / sqrtf(sum);

    for (int i = 0; i < dim; i++)
      vector[i] = vector[i] * amplitudeOverSum;
  }

  template<int dim>
  void sampleOnUnitSphere(Dtype *vector) {
    sampleVectorInNormalUniform<dim>(vector);
    Dtype sum = 0.0f;

    for (int i = 0; i < dim; i++)
      sum += vector[i] * vector[i];

    for (int i = 0; i < dim; i++)
      vector[i] = vector[i] / sqrtf(sum);
  }

  template<typename Derived>
  void shuffleSTDVector(std::vector<Derived> &order) {
    boost::variate_generator<boost::mt19937 &, boost::uniform_int<> >
        random_number_shuffler(rngGenerator, boost::uniform_int<>());
    std::random_shuffle(order.begin(), order.end(), random_number_shuffler);
  }

  /* this method is opitmized for memory use. The column should be dynamic size */
  template<typename Derived, int Rows, int Cols>
  void shuffleColumns(Eigen::Matrix<Derived, Rows, Cols> &matrix) {
    int colSize = int(matrix.cols());

    /// sampling the order
    std::vector<int> order;
    std::vector<bool> needSuffling(colSize, true);

    order.resize(colSize);
    for (int i = 0; i < colSize; i++) order[i] = i;
    shuffleSTDVector(order);
    Eigen::Matrix<Derived, Rows, 1> memoryCol(matrix.rows());

    int colID;

    for (int colStartID = 0; colStartID < colSize; colStartID++) {
      if (order[colStartID] == colStartID || !needSuffling[colStartID]) continue;

      colID = colStartID;
      memoryCol = matrix.col(colID);
      do {
        matrix.col(colID) = matrix.col(order[colID]);
        needSuffling[colID] = false;
        colID = order[colID];
      } while (colStartID != order[colID]);
      matrix.col(colID) = memoryCol;
      needSuffling[colID] = false;
    }
  }

  std::vector<unsigned> getNrandomSubsetIdx (unsigned nOfElem, unsigned nOfSubElem) {
    std::vector<unsigned> memoryIdx(nOfSubElem);
    ///// randomly sampling memory indeces
    for (unsigned i = 0; i < nOfSubElem; i++) {
      memoryIdx[i] = intRand(0, nOfElem - 1);
      for (unsigned j = 0; j < i; j++) {
        if (memoryIdx[i] == memoryIdx[j]) {
          i--;
          break;
        }
      }
    }
    return memoryIdx;
  }

  /* this method is opitmized for memory use. The column should be dynamic size */
  template<typename Derived1, int Rows1, int Cols1, typename Derived2, int Rows2, int Cols2>
  void shuffleColumnsOfTwoMatrices(Eigen::Matrix<Derived1, Rows1, Cols1> &matrix1,
                                   Eigen::Matrix<Derived2, Rows2, Cols2> &matrix2) {
//    LOG_IF(FATAL, matrix1.cols() != matrix2.cols()) << "two matrices have different number of columns";

    int colSize = int(matrix1.cols());

    /// sampling the order
    std::vector<int> order;
    std::vector<bool> needSuffling(colSize, true);

    order.resize(colSize);
    for (int i = 0; i < colSize; i++) order[i] = i;
    shuffleSTDVector(order);
    Eigen::Matrix<Derived1, Rows1, 1> memoryCol1(matrix1.rows());
    Eigen::Matrix<Derived2, Rows2, 1> memoryCol2(matrix2.rows());

    int colID;

    for (int colStartID = 0; colStartID < colSize; colStartID++) {
      if (order[colStartID] == colStartID || !needSuffling[colStartID]) continue;

      colID = colStartID;
      memoryCol1 = matrix1.col(colID);
      memoryCol2 = matrix2.col(colID);

      do {
        matrix1.col(colID) = matrix1.col(order[colID]);
        matrix2.col(colID) = matrix2.col(order[colID]);

        needSuffling[colID] = false;
        colID = order[colID];
      } while (colStartID != order[colID]);
      matrix1.col(colID) = memoryCol1;
      matrix2.col(colID) = memoryCol2;
      needSuffling[colID] = false;
    }
  }

  /* this method is opitmized for memory use. The column should be dynamic size */
  template<typename Derived1, int Rows1, int Cols1, typename Derived2, int Rows2, int Cols2, typename Derived3, int Rows3, int Cols3>
  void shuffleColumnsOfThreeMatrices(Eigen::Matrix<Derived1, Rows1, Cols1> &matrix1,
                                     Eigen::Matrix<Derived2, Rows2, Cols2> &matrix2,
                                     Eigen::Matrix<Derived3, Rows3, Cols3> &matrix3) {

    int colSize = int(matrix1.cols());

    /// sampling the order
    std::vector<int> order;
    std::vector<bool> needSuffling(colSize, true);

    order.resize(colSize);
    for (int i = 0; i < colSize; i++) order[i] = i;
    shuffleSTDVector(order);

    Eigen::Matrix<Derived1, Rows1, 1> memoryCol1(matrix1.rows());
    Eigen::Matrix<Derived2, Rows2, 1> memoryCol2(matrix2.rows());
    Eigen::Matrix<Derived3, Rows3, 1> memoryCol3(matrix3.rows());

    int colID;

    for (int colStartID = 0; colStartID < colSize; colStartID++) {
      if (order[colStartID] == colStartID || !needSuffling[colStartID]) continue;

      colID = colStartID;
      memoryCol1 = matrix1.col(colID);
      memoryCol2 = matrix2.col(colID);
      memoryCol3 = matrix3.col(colID);
      do {
        matrix1.col(colID) = matrix1.col(order[colID]);
        matrix2.col(colID) = matrix2.col(order[colID]);
        matrix3.col(colID) = matrix3.col(order[colID]);
        needSuffling[colID] = false;
        colID = order[colID];
      } while (colStartID != order[colID]);
      matrix1.col(colID) = memoryCol1;
      matrix2.col(colID) = memoryCol2;
      matrix3.col(colID) = memoryCol3;
      needSuffling[colID] = false;
    }
  }

  /* this method is opitmized for memory use. The column should be dynamic size */
  template<typename Derived1, int Rows1, int Cols1, typename Derived2, int Rows2, int Cols2, typename Derived3, int Rows3, int Cols3>
  void shuffleColumnsOfFourMatrices(Eigen::Matrix<Derived1, Rows1, Cols1> &matrix1,
                                     Eigen::Matrix<Derived2, Rows2, Cols2> &matrix2,
                                     Eigen::Matrix<Derived3, Rows3, Cols3> &matrix3,
                                     Eigen::Matrix<Derived3, Rows3, Cols3> &matrix4) {

    int colSize = int(matrix1.cols());

    /// sampling the order
    std::vector<int> order;
    std::vector<bool> needSuffling(colSize, true);

    order.resize(colSize);
    for (int i = 0; i < colSize; i++) order[i] = i;
    shuffleSTDVector(order);

    Eigen::Matrix<Derived1, Rows1, 1> memoryCol1(matrix1.rows());
    Eigen::Matrix<Derived2, Rows2, 1> memoryCol2(matrix2.rows());
    Eigen::Matrix<Derived3, Rows3, 1> memoryCol3(matrix3.rows());
    Eigen::Matrix<Derived3, Rows3, 1> memoryCol4(matrix4.rows());

    int colID;

    for (int colStartID = 0; colStartID < colSize; colStartID++) {
      if (order[colStartID] == colStartID || !needSuffling[colStartID]) continue;

      colID = colStartID;
      memoryCol1 = matrix1.col(colID);
      memoryCol2 = matrix2.col(colID);
      memoryCol3 = matrix3.col(colID);
      memoryCol4 = matrix4.col(colID);

      do {
        matrix1.col(colID) = matrix1.col(order[colID]);
        matrix2.col(colID) = matrix2.col(order[colID]);
        matrix3.col(colID) = matrix3.col(order[colID]);
        matrix4.col(colID) = matrix4.col(order[colID]);

        needSuffling[colID] = false;
        colID = order[colID];
      } while (colStartID != order[colID]);
      matrix1.col(colID) = memoryCol1;
      matrix2.col(colID) = memoryCol2;
      matrix3.col(colID) = memoryCol3;
      matrix4.col(colID) = memoryCol4;

      needSuffling[colID] = false;
    }
  }

  /*you can use this method to make the random samples the same*/
  static void seed(uint32_t seed) {
    rngGenerator.seed(seed);
  }

 private:
  static boost::random::mt19937 rngGenerator;
  static std::mutex rndMutex_;
//  boost::random::normal_distribution<Dtype> *distributionNormal;
//  boost::uniform_real<Dtype> *distributionUni, *distributionUni01;

};

}
// Initialize the random number generator with a time-based seed instead of the default one.
template<typename Dtype>
boost::random::mt19937 rai::RandomNumberGenerator<Dtype>::rngGenerator(time(NULL));
template<typename Dtype>
std::mutex rai::RandomNumberGenerator<Dtype>::rndMutex_;

#endif /* RANDOMNUMBERGENERATOR_HPP_ */
