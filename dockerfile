FROM ubuntu:24.04

# Set up python installation 
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.12-full \
    python3.12-dev \
    python3-pip
# Set this environment variable to allow pip installation to 'system' python. Virtual environments might be a more elegant solution, but that's for another time. This is my 'quick' fix.
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# GTSAM prereqs
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    g++ \
    libboost-all-dev \
    libtbb-dev \
    make \
    cmake \
    wget

# Download GTSAM
RUN cd /home && \
    wget -O gtsam.tar.gz https://github.com/borglab/gtsam/archive/82fcedf.tar.gz && \
    mkdir gtsam && \
    tar -zxf gtsam.tar.gz -C gtsam --strip-components 1 && \
    rm gtsam.tar.gz

# Install python package prereqs and any desired libraries
RUN pip install -r /home/gtsam/python/dev_requirements.txt && \
    pip install matplotlib ipython scipy

# Install/Setup GTSAM for both C++ and Python
#-DGTSAM_PYTHON_VERSION=3.12
RUN cd /home/gtsam && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DGTSAM_BUILD_PYTHON=1 && \ 
    make install -j4 && \
    # make python-install
    cd python && \
    pip install . && \
    ldconfig

# Download GPMP2 
RUN cd /home && \
    wget -O gpmp2.tar.gz https://github.com/borglab/gpmp2/archive/8be6017.tar.gz && \
    mkdir gpmp2 && \
    tar -zxf gpmp2.tar.gz -C gpmp2 --strip-components 1 && \
    rm gpmp2.tar.gz

# Install python package prereqs and any desired libraries
RUN pip install -r /home/gpmp2/python/requirements.txt

# Install/Setup GPMP2 for C++ and Python
RUN cd /home/gpmp2 && mkdir build && cd build && \
    cmake .. -DGPMP2_BUILD_PYTHON_TOOLBOX:=ON && \
    # make -j4 check && \
    make install && \
    # make python-install
    cd python && \
    pip install . && \
    ldconfig

# Copy local files to container
WORKDIR /steap

# Run python file
CMD ["python3", "main.py"]