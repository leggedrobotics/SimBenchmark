//
// Created by jemin on 10.08.16.
//

#ifndef RAI_RAI_DATA_HPP
#define RAI_RAI_DATA_HPP

#include <map>
#include <ctime>
#include <fstream>
#include "raiCommon/utils/rai_message_logger/rai_message.hpp"
#include <vector>

class RAI_data{

public:
  RAI_data(int dimension, std::string name, std::string description){
    dimension_ = dimension;
    name_ = name;
    description_ = description;
    data_.resize(dimension);
  }

  ~RAI_data(){};

  template<typename Dtype>
  void appendNewData(Dtype* data){
    int array_idx=0;
    for(int id=dimension_idx; id< dimension_; id++)
      data_[id].push_back(float(data[array_idx++]));
    dimension_idx = 0;
  }

  template<typename Dtype, typename... Args>
  void appendNewData(Dtype headValue, Args... values){
    RAIFATAL_IF(dimension_idx >= dimension_ -1,"you are trying to store more variables than the number of dimension");
    data_[dimension_idx].push_back(float(headValue));
    dimension_idx++;
    appendNewData(values...);
  }
  template<typename Dtype>
  void appendNewData(Dtype value){
    RAIFATAL_IF(dimension_idx != dimension_ -1,"You are missing some data");
    data_[dimension_idx].push_back(float(value));
    dimension_idx = 0;
  }

  ///// for pointers and values
  template<typename Dtype, typename... Args>
  void appendNewData(Dtype headValue, Args... values, Dtype* data){
    RAIFATAL_IF(dimension_idx >= dimension_ -1,"you are trying to store more variables than the number of dimension");
    data_[dimension_idx].push_back(float(headValue));
    dimension_idx++;
    appendNewData(values..., data);
  }
  template<typename Dtype>
  void appendNewData(Dtype value, Dtype* data){
    data_[dimension_idx].push_back(float(value));
    dimension_idx++;
    appendNewData(data);
  }

  const std::vector<std::vector<float> >& getData(){
    return data_;
  }

  void clearData(){
    data_.clear();
    data_.resize(dimension_);
  }

  void write2File(std::ofstream& logFile){
    logFile<<"// "<<name_<< "\n";
    logFile<<"// "<<description_<< "\n";
    for(int timeID = 0; timeID < data_[0].size(); timeID++) {
      for (int dimensionID = 0; dimensionID < dimension_; dimensionID++)
        logFile << std::setprecision(9) << data_[dimensionID][timeID] << "  ";
      logFile<<"\n";
    }
    logFile<<"\n";
  }

  std::string getName () {
    return name_;
  }

private:
  int dimension_ = 1;
  std::string name_;
  std::string description_;
  std::vector<std::vector<float> > data_;
  int dimension_idx = 0;

};

#endif //RAI_RAI_DATA_HPP
