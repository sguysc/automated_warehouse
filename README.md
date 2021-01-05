Installation 
============

## Install slugs

`git clone -b LTL_stack https://git@github.com/wongkaiweng/slugs.git`

`cd slugs/src`

`make`

## Install Python dependencies

`pip install dill`

`pip install networkx`

`pip install tqdm`

`pip install shapely`

`pip install python-fcl`

`pip install subprocess32`

(

`pip install --upgrade numpy`

`pip install -U matplotlib==2.1.2`

)

Don't forget to install ros-kinetic on the computer http://wiki.ros.org/kinetic/Installation

Install the Jackal ROS packages: https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html

Install multi_jackal for simulations: https://github.com/NicksSimulationsROS/multi_jackal

(

Install vicon_bridge if needed to run with physical robots

)

## Download this repo

`git clone -b multi_robot https://github.com/sguysc/automated_warehouse.git`


Running 
=======

To create funnels, use example:

run `DubinsPlantCar.py`

To create a library of motions:

run `CreateMPLibrary.py`

To create a single specification, see:

run `warehouse_map.py`

Or, you could simply use the GUI app to create specifications, synthesize them and run them

in simulation or on real robots using ROS:

run `GUI.py`

Simulation
==========

Open terminal, run `roscore`

Open terminal, navigate to `simulation/jackal_nav` and run `runme.bash`

Open terminal, run `./GUI.py` or `./Jackal.py`


