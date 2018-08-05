FROM ubuntu:16.04

RUN apt-get update -qq && apt-get -y -qq upgrade

# essentials
RUN apt-get install -y -qq build-essential git software-properties-common

# cmkae 
RUN apt-get install -y -qq cmake

# g++ 
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y -qq g++-7 g++-6

# other dependencies 
RUN apt-get install -y -qq libeigen3-dev libboost-all-dev libglew-dev libglm-dev libsdl2-dev \
libassimp-dev libsoil-dev libsdl2-ttf-dev

# rai common dependencies
RUN apt-get install -y -qq gnuplot5

# rai graphics dependencies
RUN apt-get install -y -qq libsdl2-image-dev ffmpeg