#!/usr/bin/env bash

cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make
