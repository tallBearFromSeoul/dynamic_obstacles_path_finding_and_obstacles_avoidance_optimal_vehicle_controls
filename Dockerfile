FROM ubuntu:18.04
LABEL Description="C++ Build environment"
ENV HOME /root
ARG DEBIAN_FRONTEND=noninteractive
COPY coinhsl-archive-2021.05.05.tar.gz source_files.zip ${HOME}
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get -y install \
		vim build-essential clang gcc g++ gfortran patch unzip pkg-config libmetis-dev cmake gdb wget git software-properties-common\
		libeigen3-dev libsuitesparse-dev \
		libglm-dev libassimp-dev libglfw3 libglfw3-dev libssl-dev \
		libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev \
		libc++-dev libglew-dev g++ ninja-build \
		libglu1-mesa-dev freeglut3-dev mesa-common-dev \
		libjpeg-dev libpng-dev \
		libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
		libdc1394-22-dev libraw1394-dev libopenni-dev 

RUN cd ${HOME} && wget http://www.cmake.org/files/v3.23/cmake-3.23.0.tar.gz && \
		tar xzf cmake-3.23.0.tar.gz && \
		cd cmake-3.23.0 && mkdir build && cd build && \
		../configure && make && make install && \
		cd ${HOME} && rm -rf cmake-3.23.0.tar.gz cmake-3.23.0

RUN cd ${HOME} && gunzip coinhsl-archive-2021.05.05.tar.gz && \
		tar xf coinhsl-archive-2021.05.05.tar && \
		git clone https://github.com/coin-or-tools/ThirdParty-HSL.git && \
		cd ThirdParty-HSL && mv ../coinhsl-archive-2021.05.05 ./coinhsl

RUN	cd ${HOME}/ThirdParty-HSL && mkdir build && cd build && \
		../configure && make && make install && \
		cd ${HOME} && rm -rf coinhsl-archive-2021.05.05.tar ThirdParty-HSL

RUN cd ${HOME} && \
		git clone https://github.com/coin-or/Ipopt && \
		cd Ipopt && mkdir build && cd build && \
		../configure && make && make test && make install && \
		cd ${HOME} && rm -rf Ipopt	

RUN cd ${HOME} && \
		git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
		cd Pangolin && mkdir build && cd build && \
		cmake .. && make && make install && \
		cd ${HOME} && rm -rf Pangolin

RUN cd ${HOME} && \
		git clone https://github.com/coin-or/CppAD.git && \
		cd CppAD && mkdir build && cd build && \
		cmake .. && make check && make install && \
		cd ${HOME} && rm -rf CppAD

RUN cd ${HOME} && \
		git clone https://github.com/nothings/stb && \
		cd stb && cp -a *.h /usr/local/include/ && \
		cd ${HOME} && rm -rf stb

RUN cd ${HOME} && \
		git clone https://github.com/Dav1dde/glad.git && \
		cd glad && mkdir build && cd build && \
		cmake .. && make && \
		cp -a include /usr/local/include && \
		cp -a libglad.a /usr/local/lib && \
		cd ${HOME} && rm -rf glad

RUN add-apt-repository ppa:deadsnakes/ppa && \
		apt-get -y install \
		python3.9 python3.9-dev python3.9-distutils

RUN cd ${HOME} && unzip source_files.zip && \
		cd mpc_demo && mkdir build && cd build && \
		cmake .. && make && \
		cd ${HOME} && rm -rf source_files.zip

