#!/usr/bin/env bash

cd $TRAVIS_BUILD_DIR
mkdir build
echo $PWD
cd build
echo $PWD
cmake -DCMAKE_BUILD_TYPE=Release ../
make
