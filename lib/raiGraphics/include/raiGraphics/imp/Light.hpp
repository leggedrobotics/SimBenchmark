//
// Created by jhwangbo on 17. 4. 28.
//

#ifndef PROJECT_LIGHT_HPP
#define PROJECT_LIGHT_HPP

#include "vector"
#include "Eigen/Core"
#include "mutex"
#include "iostream"

namespace rai_graphics{

class Light{

 public:
  void getPosition(std::vector<float>& pos){
    mtx.lock();
    pos = position;
    mtx.unlock();
  }

  void getAmbient(std::vector<float>& amb){
    mtx.lock();
    amb = ambient;
    mtx.unlock();
  }

  void getDiffuse(std::vector<float>& diff){
    mtx.lock();
    diff = diffuse;
    mtx.unlock();
  }

  void getSpecular(std::vector<float>& spec){
    mtx.lock();
    spec = specular;
    mtx.unlock();
  }

  void setPosition(std::vector<float>& pos){
    mtx.lock();
    position = pos;
    mtx.unlock();
  }

  void setAmbient(std::vector<float>& amb){
    mtx.lock();
    ambient = amb;
    mtx.unlock();
  }

  void setDiffuse(std::vector<float>& diff){
    mtx.lock();
    diffuse = diff;
    mtx.unlock();
  }

  void setSpecular(std::vector<float>& spec){
    mtx.lock();
    specular = spec;
    mtx.unlock();
  }

 private:
  std::vector<float> position = {-100.0,0.0,10.0};
  std::vector<float> ambient = {0.5,0.5,0.5};
  std::vector<float> diffuse = {1,1,1};
  std::vector<float> specular = {0.7,0.7,0.7};
  std::mutex mtx;
};

} // rai_graphics

#endif //PROJECT_LIGHT_HPP
