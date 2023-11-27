# This should be done only once

# 1. Cloning the git repository
git clone https://github.com/a2s-institute/master-board.git
echo "The [Git-Repo] is cloned successfully"

# 2. Initial setup of the master-board SDK
cd master-board/sdk/master_board_sdk

# 3. Creating a build folder to store compiled data 
mkdir build && cd build
echo "[Initialzing] the cmake "

# 4. It is used to create a list, which checks for the necessasry files
cmake -DBUILD_PYTHON_INTERFACE=ON _DCMAKE_BUILD_TYPE=RELEASE ..
echo "[Check] for Error Statements, if not Yay!! --> to next step compiling"
# 5. For compiling the script  that is there in the folder
make

# 6. If there is no make installed in your system
sudo make install
echo "Initializing the process of installing [GAZEBO]"
# 7. For the Gazebo Installation
curl -sSL http://get.gazebosim.org | sh


