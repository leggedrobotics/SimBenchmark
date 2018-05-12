//
// Created by kangd on 12.05.18.
//

#include <iostream>
#include <ode/ode.h>

int main() {
  dReal w[3] = {1, 2, 3};
  dMatrix3 A;
  A[0] = 11;
  A[1] = 12;
  A[2] = 13;
  A[3] = 0;
  A[4] = 21;
  A[5] = 22;
  A[6] = 23;
  A[7] = 0;
  A[8] = 31;
  A[9] = 32;
  A[10] = 33;
  A[11] = 0;

  dMatrix3 B;
  B[0] = 1;
  B[1] = 2;
  B[2] = 3;
  B[3] = 0;
  B[4] = 4;
  B[5] = 5;
  B[6] = 6;
  B[7] = 0;
  B[8] = 7;
  B[9] = 8;
  B[10] = 9;
  B[11] = 0;

  dReal wTw = dDot(w, w, 3);
  std::cout << "----" << std::endl;
  std::cout << wTw << std::endl;  // should be 14

  dReal temp[3];
  dMultiply0_331(temp, A, w);
  std::cout << "----" << std::endl;
  std::cout << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;  // should be [74, 134, 194]

  dReal wTAw = dDot(w, temp, 3);
  std::cout << "----" << std::endl;
  std::cout << wTAw << std::endl;  // should be 924

  dMatrix3 tempMat;
  dMultiply0_333(tempMat, A, B);
  std::cout << "----" << std::endl;
  std::cout << tempMat[0] << " " << tempMat[1] << " " << tempMat[2] << std::endl; // [74, 182, 290]
  std::cout << tempMat[4] << " " << tempMat[5] << " " << tempMat[6] << std::endl; // [134, 332, 530]
  std::cout << tempMat[8] << " " << tempMat[9] << " " << tempMat[10] << std::endl;// [194, 486, 770]
}