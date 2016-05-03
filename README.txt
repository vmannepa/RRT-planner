THIS FILE SPECIFIES HOW TO RUN THE MOTION PLANNER PYTHON CODE IN LINUX ENVIRONMENT 
AND USING  PYGAME INTERFACE:

INSTALL PYTHON 2.7.x:

#First, install some dependencies:
sudo apt-get install build-essential checkinstall
sudo apt-get install libreadline-gplv2-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev

#Then download using the following command:

cd ~/Downloads/
wget http://python.org/ftp/python/2.7.5/Python-2.7.5.tgz

#Extract and go to the dirctory:
tar -xvf Python-2.7.5.tgz
cd Python-2.7.5

#Now, install using the command you just tried:

./configure
make
sudo checkinstall

Installing pygame using PYTHON 2.7.X

#install dependencies
sudo apt-get install mercurial python-dev python-numpy libav-tools \
    libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsmpeg-dev \
    libsdl1.2-dev  libportmidi-dev libswscale-dev libavformat-dev libavcodec-dev
 
# Grab source
hg clone https://bitbucket.org/pygame/pygame
 
# Finally build and install
cd pygame
python setup.py build
sudo python setup.py install

PROGRAN DESIGN:
Program basically is designed using all the functions that are required to implement the motion planner for RRT 

1. dist()         Distance Funcion : Calculate Eculidian Distance between two points
2. runge_k_4()    Runge kutta method : Used for curve smoothing and impelments non-holonomic constraints
3. config_space() Configuration space : Calculates configuration space for obstacle using robot configuration resulting in Cfree.
4. pointin_poly() Collision detection : Uses ray casting method for collision detection
5. main()         Motion Planner      : RRT is implemented here


NOW YOU ARE READY TO EXECUTE THE PYTHON FILES

1. RRT_VEERA_ENV1_FRONT.PY
   This python file has the motion planner developed for Environment 1 using Front wheel constraints
   use command from Terminal : python rrt_veera_env1_front.py
2. RRT_VEERA_ENV1_REAR.PY
   This python file has the motion planner developed for Environment 1 using REAR wheel constraints
   use command from Terminal : python rrt_veera_env1_rear.py
3. RRT_VEERA_ENV2_FRONT.PY
   This python file has the motion planner developed for Environment 2 using Front wheel constraints
   use command from Terminal : python rrt_veera_env2_front.py
4. RRT_VEERA_ENV2_REAR.PY
   This python file has the motion planner developed for Environment 2 using REAR wheel constraints
   use command from Terminal : python rrt_veera_env2_rear.py



