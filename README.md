# ConCPP (Concurrent Coverage Path Planning)
Implementation of **Online Concurrent Multi-Robot Coverage Path Planning** [IROS 2025] in **ROS Noetic Ninjemys** using **C++**.

Paper's URL: [IEEEXplore](https://ieeexplore.ieee.org/abstract/document/) and [CoRR](https://arxiv.org/abs/2403.10460)<br/>
Video's URL: [YouTube](https://www.youtube.com/watch?v=)

#### Instructions:

1.  Download the source code package:<br/> 
    `cd ~/catkin_ws/src/`<br/> 
    `git clone https://github.com/iitkcpslab/ConCPP.git`
2.  Understand the directory structure:<br/> 
    1.  include: Contains the header files.<br/> 
        By default, the package is configured for the motions of a *Quadcopter in a 2D workspace*. However, this can be reconfigured for the motions of a *Turtlebot* in *config.h* file by uncommenting `#define TURTLEBOT`. 
    2.  input: *ws_obs_robs.txt* represents the 2D workspace grid with obstacles and initial locations of the robots. 
         | Value      | Meaning                              |
         | -----      | ------------------------------------ |
         | 0.0        | Obstacle-occupied cell               |
         | 0.5        | Obstacle-free cell                   |
         | i (&ge; 1) | Initial location of **Robot-i**      |

        *E.g.*, the given file represents a *10 x 10* grid with 3 robots - Robot-1, Robot-2, and Robot-3, having initial locations (1,0), (2, 1), and (3,2), respectively. 
    3.  msg: Contains message files. 
    4.  output: Contains the obtained results when run with the input.
    5.  rviz: Contains the configuration file of rviz. 
    6.  src: Contains the source files corresponding to the header files. 
        * Robot side: *robot.cpp* emulates a robot. 
        * Coverage Planner side: The rest of the source files. 
    7.  srv: Contains the service files. 
3.  Build the package:<br/> 
    `cd ~/catkin_ws && catkin_make clean && catkin_make`<br/>
    `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`<br/>
    `mkdir ~/catkin_ws/src/ConCPP/output`
4.  Run the package in a Terminal:
    -   Tab 1:
        `rosclean purge -y && pkill roscore ; roscore`
    -   Tab 2:
    	`clear && rosrun rviz rviz -d ~/catkin_ws/src/ConCPP/rviz/concpp.rviz`
    -   Tab 3:
        `rm ~/catkin_ws/src/OnDemCPP/output/*`<br/> 
        `rosrun con_plan_exec_pkg conCpExe _ws_x:=<Workspace size along the +x axis> _ws_y:=<Workspace size along the +y axis> _rc:=<Robot count>`
    -   Tab 4:
        `cp ~/<Workspace directory>/ws_obs_robs.txt ~/catkin_ws/src/OnDemCPP/input/`<br/>
        `rosrun con_plan_exec_pkg conRobotExe __name:=robot_<Robot ID> _rid:=<Robot ID> # Run for each Robot-i`