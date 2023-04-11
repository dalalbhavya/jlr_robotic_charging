# JLR Robotic Charging Challenge
Inter IIT 11.0 Mid-Prep Problem Statement

[Problem Statement](/Mid_JLR%20copy.pdf)

# Setup
* `git clone git@github.com:ros-industrial/universal_robot.git`
* `rosdep update`
* `rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src`
* `roslaunch ur_gazebo ur5_bringup.launch`


# Tasks 
- [x] Design and develop a simulated environment.
- [x] Detect the location of the charging port on the vehicle
    - [x] CCS2 plug detection
- [ ] Avoiding sharp bending angles for charging cable.
    - [ ] Determine the cable size and feasible curvature. 
- [x] Implement mechanical Constraints
    - [x] Charger plugging operation speed should'nt exceed 0.1 m/s
    - [x] Rotation Speed should not exceed 5 deg/sec
- [x] Easy to change start and end pose of end-effector.

 
