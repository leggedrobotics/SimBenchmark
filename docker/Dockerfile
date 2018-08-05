FROM ubuntu:16.04

RUN apt-get update -qq && apt-get -y -qq upgrade

# essentials
RUN apt-get install -y -qq build-essential git software-properties-common

# cmake
RUN apt-get install -y -qq cmake

# g++ 
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y -qq g++-7 g++-6

# other dependencies 
RUN apt-get install -y -qq libeigen3-dev libboost-all-dev libglew-dev libglm-dev libsdl2-dev \
libassimp-dev libsoil-dev libsdl2-ttf-dev

# raicommon dependencies
RUN apt-get install -y -qq gnuplot5

# raigraphics dependencies
RUN apt-get install -y -qq libsdl2-image-dev ffmpeg

# urdf-dom
RUN apt-get install -y -qq liburdfdom-dev

# dart dependencies
RUN apt-get remove -y libdart*
RUN apt-get install -y libassimp-dev libccd-dev libfcl-dev
RUN apt-get install -y libopenscenegraph-dev