//
// Created by kangd on 11.02.18.
//

#include "Base_RG.hpp"
benchmark::Base_RG::~Base_RG() {
  if(!isEnded_ && isReady_)
    visEnd();
}
