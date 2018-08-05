# SimBenchmark Docker 

You can either build a docker image from Dockerfile or pull the image from docker repository

1. Get docker image
	- option 1: Pull from repository (recommended)
		```sh 
		$ docker pull donghokang/simbenchmark 
		```
	- option 2: Build from Dockerfile
		- 1. build docker file
		```sh 
		$ docker build -f ./Dockerfile -t donghokang/simbenchmark:latest .
		```
		- 2. install dart 6.4.0 from source file 
2. Run docker container from the image   
	```sh
	$ docker run -it --rm -v <SimBenchmark root dir>:/home/simbench donghokang/simbenchmark:latest
	```

## Note 

- You may need the super user privilege for run docker commands. 
	- If you don't want to use sudo, see [this page](https://docs.docker.com/install/linux/linux-postinstall/) 
- You can use the following command to mount a host directory and forward display for using GUI  
	- run ```xhost local:root``` on the host machine.
	- run docker with the following options  
		```sh
		$ docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --device /dev/dri donghokang/simbenchmark:latest
		```
		- ```-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY --device /dev/dri```: Read [this](http://somatorio.org/en/post/running-gui-apps-with-docker/)  