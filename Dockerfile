FROM ubuntu:22.04

WORKDIR /solo12_robot

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
        curl \
        wget \
        gnupg \
        build-essential \
        cmake \
        python3 \
        python-is-python3 \
        libboost-python-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN git clone https://github.com/a2s-institute/master-board.git

RUN cd master-board/sdk/master_board_sdk \
    && mkdir build \
    && cd build \
    && cmake -DBUILD_PYTHON_INTERFACE=ON _DCMAKE_BUILD_TYPE=RELEASE .. \
    && make \
    && make install

RUN curl -sSL http://get.gazebosim.org | sh
