FROM omnetpp/omnetpp:u18.04-5.4.1 

WORKDIR /home

ADD . /home/artery

# For openscenegraph add the following ppa
RUN apt update && apt-get -y install software-properties-common && add-apt-repository ppa:openmw/openmw

# Required packets  TODO geographical select
RUN apt update && \
	 DEBIAN_FRONTEND=noninteractive apt -y -f install make git qt5-default build-essential gcc g++ bison flex perl tcl-dev tk-dev blt libxml2-dev zlib1g-dev default-jre doxygen graphviz libwebkitgtk-1.0-0 openmpi-bin libopenmpi-dev libpcap-dev autoconf automake libtool libproj-dev libfox-1.6-dev libgdal-dev libxerces-c-dev qt4-dev-tools openscenegraph-3.4 libosgearth-dev libboost-all-dev libgeographic-dev libcrypto++-dev libavcodec-dev libavformat-dev r-base libswscale-dev default-jdk default-jre libgtest-dev swig

# Reconfigure and recompile omnetpp
RUN cd /root/omnetpp && ./configure && make

# Get fresh cmake TODO export PATH
RUN mkdir /home/cmake_dir && cd /home/cmake_dir && wget https://github.com/Kitware/CMake/releases/download/v3.13.3/cmake-3.13.3-Linux-x86_64.tar.gz && tar -xvzf cmake-3.13.3-Linux-x86_64.tar.gz

# Set environmental variable
ENV CMAKE_HOME=/home/cmake_dir/cmake-3.13.3-Linux-x86_64/bin
ENV PATH=$PATH:/home/cmake_dir/cmake-3.13.3-Linux-x86_64/bin

# Get SUMO
RUN cd /home && git clone --recursive https://github.com/eclipse/sumo && cd /home/sumo && git fetch origin refs/replace/*:refs/replace/* && mkdir build/cmake-build && cd build/cmake-build && cmake ../.. && make -j 8 && make install

# Export env var
ENV SUMO_HOME=/home/sumo

# Get artery and build it 
RUN cd /home/artery && mkdir -p build && rm -f build/CMakeCache.txt && make all && cd build && cmake -DWITH_CMDENV=ON .. && cmake -DWITH_CMDENV=ON --build .

ENV SCENARIO run_car2car-grid 

CMD cd /home/artery && cmake --build build --target $SCENARIO
