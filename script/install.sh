# This should be done only once

echo "#############################"
echo "# Install Dependencies      #"
echo "#############################"

sudo apt-get update

sudo apt-get install -y git \
    curl \
    wget \
    python3 \
    python-is-python3 \
    libboost-python-dev

echo "#############################"
echo "# Master Board              #"
echo "#############################"

git clone https://github.com/a2s-institute/master-board.git
if [ $? -eq  0 ]; then
    echo "The [Git-Repo] is cloned successfully"
else
    echo "[Fail] Could not git clone master_board_sdk"
    exit 1
fi

cd master-board/sdk/master_board_sdk

mkdir build && cd build

cmake -DBUILD_PYTHON_INTERFACE=ON _DCMAKE_BUILD_TYPE=RELEASE ..
if [ $? -eq  0 ]; then
    echo "[Build] successfully builded master_board_sdk"
else
    echo "[Fail] Could not build. check output."
    exit 1
fi

make

sudo make install
if [ $? -eq  0 ]; then
    echo "[Install] successfully installed master_board_sdk"
else
    echo "[Fail] Could not install. check output."
    exit 1
fi

echo "#############################"
echo "# Gazebo Simulator          #"
echo "#############################"
# 7. For the Gazebo Installation
curl -sSL http://get.gazebosim.org | sh
