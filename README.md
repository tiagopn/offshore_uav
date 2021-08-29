Installation instructions:
  - Follow the installation processes at https://github.com/ctu-mrs/mrs_uav_system;
  - Setup and install dependencies 
```
sudo apt-get update
sudo apt-get install libfftw3-dev
sudo apt-get install libclfft-dev
sudo apt-get install libcgal-dev
```
  - In the home/workspace/src directory clone this package;
  - Run a catkin build;
  - Using terminal access home/workspace/src/offshore_uav/offshore_uav_pack/start;
  - Run this command on the terminal:
  
    $ ./start.sh
  - Before "starting" the simulation, just run the second script in the tmux spawn window, to spawn the USV
  - Now press play on the gazebo.
