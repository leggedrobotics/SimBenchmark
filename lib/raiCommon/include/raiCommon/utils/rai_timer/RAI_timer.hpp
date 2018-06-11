//
// Created by jhwangbo on 11.11.16.
// this method is relatively accurate down to 1 micro second.
//

#ifndef RAI_RAI_TIMER_HPP
#define RAI_RAI_TIMER_HPP

//#define RAI_TIMER_WALL_CLOCK

#include <vector>
#include <float.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <boost/filesystem.hpp>
#include "RAI_timer_items.hpp"
#include "raiCommon/utils/rai_message_logger/rai_message.hpp"

#ifdef RAI_TIMER_WALL_CLOCK
#include <sys/time.h>
#else
#include <time.h>
#define RAI_CLOCK_TYPE CLOCK_MONOTONIC
#endif

namespace rai {
namespace Utils {

class Timer {
 public:

  typedef int TimerItem;

  Timer() : items_("Computation", -1) {
#ifdef RAI_TIMER_WALL_CLOCK
    gettimeofday(&timevalNow, nullptr);
    processStartTime_ = timevalNow.tv_sec + 1e-6 * timevalNow.tv_usec;
#else
    clock_gettime(RAI_CLOCK_TYPE, &timespec);
    processStartTime_ = timespec.tv_sec + 1e-9 * timespec.tv_nsec;
#endif
    file_name_ = "timerLog";
  }

  ~Timer() {
    std::ostringstream logPath;
    logPath << log_path_ << "/" << file_name_ << ".rlog";

    // create path directories if not exists
    boost::filesystem::path dirPath(log_path_.c_str());

    if(!boost::filesystem::exists(dirPath))
      boost::filesystem::create_directories(dirPath);

    std::ofstream logFile;
    logFile.open(logPath.str().c_str());
    logFile << "logging in seconds" << "\n";
    for (int idx = 0; idx < name_.size(); idx++)
      logFile << std::setw(30) << std::left << name_[idx] << "|" << "  N of samples: " << std::setw(8) << std::left
              << frameCount[idx] << "  " << "min: " << std::setw(15) << std::left << timeMin[idx] << "max: "
              << std::setw(15) << std::left << timeMax[idx] << "Avg: " << std::setw(15) << std::left << timeSum[idx] / (float) frameCount[idx]
              << "Total: " << std::setw(15) << std::left << timeSum[idx] << "\n";
    logFile.close();
  }

  void dumpToStdOuput() {
    std::ostringstream loggingdata;
    for (int idx = 0; idx < name_.size(); idx++)
      loggingdata << std::setw(30) << std::left << name_[idx] << "|" << "  N of samples: " << std::setw(8) << std::left
                  << frameCount[idx] << "  " << "min: " << std::setw(15) << std::left << timeMin[idx] << "max: "
                  << std::setw(15) << std::left << timeMax[idx] << "Avg: " << std::setw(15) << std::left << timeSum[idx] / (float) frameCount[idx]
                  << "Total: " << std::setw(15) << std::left << timeSum[idx] << "\n";
    std::cout << loggingdata.str() << "\n";
  }

  TimerItem createTimedItem(std::string name) {
    name_.push_back(name);
    timeMin.push_back(DBL_MAX);
    timeMax.push_back(0.0);
    timeSum.push_back(0.0);
    frameCount.push_back(0);
    curTime.push_back(0.0);
    curTime_ns.push_back(0.0);
    allParent.clear();

    for (int itemID = 0; itemID < name_.size(); itemID++)
      if (curTime[itemID] != 0)
        allParent.push_back(itemID);

    items_.add_item(name, name_.size() - 1, allParent);

    return name_.size() - 1;
  }

  void startTimer(TimerItem item) {
//    if (disableTimer) return;
    clock_gettime(RAI_CLOCK_TYPE, &timespec);
    curTime[item] = timespec.tv_sec;
    curTime_ns[item] = timespec.tv_nsec;
  }

  void stopTimer(TimerItem item) {
//    if (disableTimer) return;
    clock_gettime(RAI_CLOCK_TYPE, &timespec);
    timeLaps = timespec.tv_sec - curTime[item] + 1e-9 * (timespec.tv_nsec - curTime_ns[item]);
    timeSum[item] += timeLaps;
    timeMin[item] = (timeMin[item] > timeLaps) ? timeLaps : timeMin[item];
    timeMax[item] = (timeMax[item] > timeLaps) ? timeMax[item] : timeLaps;
    frameCount[item]++;
  }

  void startTimer(const std::string& name) {
    if (disableTimer) return;
    unsigned int idx = std::find(name_.begin(), name_.end(), name) - name_.begin();
    if (idx == name_.size()) {
      name_.push_back(name);
      timeMin.push_back(DBL_MAX);
      timeMax.push_back(0.0);
      timeSum.push_back(0.0);
      frameCount.push_back(0);
      curTime.push_back(0.0);
      curTime_ns.push_back(0.0);
      allParent.clear();

      for (int itemID = 0; itemID < name_.size(); itemID++)
        if (curTime[itemID] != 0)
          allParent.push_back(itemID);

      items_.add_item(name, idx, allParent);
    }

    DRAIWARN_IF(curTime[idx] != 0.0, name << ": start timer is called twice consecutively. Call stopTimer first");
#ifdef RAI_TIMER_WALL_CLOCK
    gettimeofday(&timevalNow, nullptr);
    curTime[idx] = timevalNow.tv_sec + 1e-6 * timevalNow.tv_usec;
#else
    clock_gettime(RAI_CLOCK_TYPE, &timespec);
    curTime[idx] = timespec.tv_sec;
    curTime_ns[idx] = timespec.tv_nsec;
#endif
  }

  void stopTimer(const std::string& name) {
    if (disableTimer) return;
#ifdef RAI_TIMER_WALL_CLOCK
    gettimeofday(&timevalNow, nullptr);
#else
    clock_gettime(RAI_CLOCK_TYPE, &timespec);
#endif
    unsigned int idx = std::find(name_.begin(), name_.end(), name) - name_.begin();
    if (idx == name_.size()) {
      DRAIWARN("stop timer is called before start timer");
      return;
    } else {
#ifdef RAI_TIMER_WALL_CLOCK
      timeLaps = timevalNow.tv_sec + 1e-6 * timevalNow.tv_usec - curTime[idx];
#else
      timeLaps = timespec.tv_sec - curTime[idx] + 1e-9 * (timespec.tv_nsec - curTime_ns[idx]);
#endif
      curTime[idx] = 0.0;
    }

    timeSum[idx] += timeLaps;
    timeMin[idx] = std::min(timeMin[idx], timeLaps);
    timeMax[idx] = std::max(timeMax[idx], timeLaps);
    frameCount[idx]++;
  }

  std::string getCurrentDataAndTime() {
    std::ostringstream dataAndTime;
    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);
    dataAndTime << (now->tm_year + 1900) << '-' << std::setfill('0') << std::setw(2) << (now->tm_mon + 1) << '-'
                << std::setw(2) << now->tm_mday << '-' << std::setw(2) << now->tm_hour << '-' << std::setw(2)
                << now->tm_min << '-' << std::setw(2) << now->tm_sec;
    return dataAndTime.str().c_str();
  }

  std::vector<double> &getTotalTime() {
    return timeSum;
  }

  std::vector<std::string> &getNames() {
    return name_;
  }

  Timer_items &getTimedItems() {
    for (int i = 0; i < name_.size(); i++)
      items_.update_time_sum_(i, timeSum[i]);

    double totalTime = 0;
    for (int i = 0; i < items_.subItems_.size(); i++)
      totalTime += items_.subItems_[i].time_sum_;

    items_.update_time_sum_(-1, totalTime);

    return items_;
  }

  void setLogPath(const std::string& path){
    log_path_ = path;
  }

  void setLogFileName(const std::string& name) {
    file_name_ = name;
  }

  void disable() {
    disableTimer = true;
  }

  void enable() {
    disableTimer = false;
  }

  double getGlobalElapsedTimeInSec() {
    double currentTime;
#ifdef RAI_TIMER_WALL_CLOCK
    gettimeofday(&timevalNow, nullptr);
    currentTime = timevalNow.tv_sec + 1e-6 * timevalNow.tv_usec;
#else
    clock_gettime(RAI_CLOCK_TYPE, &timespec);
    currentTime = timespec.tv_sec + 1e-9 * timespec.tv_nsec;
#endif
    return currentTime - processStartTime_;
  }

  double getGlobalElapsedTimeInMin() {
    double currentTime;
#ifdef RAI_TIMER_WALL_CLOCK
    gettimeofday(&timevalNow, nullptr);
    currentTime = timevalNow.tv_sec + 1e-6 * timevalNow.tv_usec;
#else
    clock_gettime(RAI_CLOCK_TYPE, &timespec);
    currentTime = timespec.tv_sec + 1e-9 * timespec.tv_nsec;
#endif
    return (currentTime - processStartTime_) / 60.0;
  }

  double getGlobalElapsedTimeInHr() {
    double currentTime;
#ifdef RAI_TIMER_WALL_CLOCK
    gettimeofday(&timevalNow, nullptr);
    currentTime = timevalNow.tv_sec + 1e-6 * timevalNow.tv_usec;
#else
    clock_gettime(RAI_CLOCK_TYPE, &timespec);
    currentTime = timespec.tv_sec + 1e-9 * timespec.tv_nsec;
#endif
    return (currentTime - processStartTime_) / 3600.0;
  }

 private:
  std::vector<std::string> name_;
  timeval timevalNow;
  struct timespec timespec;
  std::vector<double> timeSum;
  std::vector<double> timeMin;
  std::vector<double> timeMax;
  std::vector<int> frameCount;
  std::vector<int> allParent;
  std::vector<double> curTime;
  std::vector<double> curTime_ns;
  double timeLaps;
  std::string log_path_ = "/tmp";
  std::string file_name_;
  Timer_items items_;
  bool disableTimer = false;
  double processStartTime_;

};

}
}

#endif //RAI_RAI_TIMER_HPP
