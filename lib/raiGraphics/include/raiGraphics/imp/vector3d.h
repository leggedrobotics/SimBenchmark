#ifndef VECTOR3D_H
#define VECTOR3D_H
#include <iostream>
#include <cmath>

class vector3d{
 public:
  float x,y,z;
  vector3d();
  vector3d(float a,float b);
  vector3d(float a,float b,float c);

  float dotProduct(const vector3d& vec2);
  vector3d crossProduct(const vector3d& vec2);
  float length();
  void normalize();

  void change(float a,float b,float c);
  void change(vector3d vec2);
  void changeX(float a);
  void changeY(float a);
  void changeZ(float a);

  vector3d operator+(const vector3d& vec2);
  vector3d operator-(const vector3d& vec2);
  vector3d operator*(float num);
  vector3d operator/(float num);

  vector3d& operator+=(const vector3d& vec2);
  vector3d& operator-=(const vector3d& vec2);
  vector3d& operator*=(float num);
  vector3d& operator/=(float num);

  bool operator==(const vector3d vec2);
  bool operator!=(const vector3d vec2);

  friend std::ostream& operator<<(std::ostream& out,const vector3d& vec);
};

#endif