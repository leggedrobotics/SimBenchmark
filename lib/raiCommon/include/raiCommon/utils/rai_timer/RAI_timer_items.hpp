//
// Created by jhwangbo on 08.12.16.
//

#ifndef RAI_RAI_TIMER_ITEMS_HPP
#define RAI_RAI_TIMER_ITEMS_HPP
#include <string>
#include <algorithm>

namespace rai {
namespace Utils {

class Timer_items {

 public:

  Timer_items(std::string name, int id) : name_(name), id_(id) {

  }

  void add_item(const std::string& name, int id, std::vector<int> parentsID) {
    totalChildren_++;

    if (parentsID.size() == 0) {
      subItems_.push_back(Timer_items(name, id));
      return;
    }
    maximum_depth = std::max(maximum_depth, int(parentsID.size()+1));


    for (int i = 0; i < subItems_.size(); i++) {
      unsigned int idx = std::find(parentsID.begin(), parentsID.end(), subItems_[i].id_) - parentsID.begin();
      if(idx!=parentsID.size()) {
        parentsID.erase(parentsID.begin() + idx);
        subItems_[i].add_item(name, id, parentsID);
        break;
      }
    }
  }

  void update_time_sum_(int id, double time_sum) {
    if (id_ == id) {
      time_sum_ = time_sum;
      return;
    }

    for (int i = 0; i < subItems_.size(); i++) {
      subItems_[i].update_time_sum_(id, time_sum);
    }
  }

  std::string name_;
  double time_sum_;
  int id_;
  std::vector<Timer_items> subItems_;
  int totalChildren_ = 0;
  int maximum_depth = 0;
};

}
}
#endif //RAI_RAI_TIMER_ITEMS_HPP

