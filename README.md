# Robot Simulation FSM Based Controller using C++
Develop FSM(Finite State Machine) based controller and follow the yellow robot in the environment.

Develop FSM(Finite State Machine) based controller and follow the yellow robot in the environment:<br>
(1) It is necessary to develop a follower controller based on the desired colour of the robot. Be careful while approaching the yellow robot; you must slow down.<br>
(2) When the robot loses the yellow robot, the controller must use a wander controller to search for it.<br>
(3)You must implement an obstacle avoidance controller, but also consider the detected robot's colour because the detected robot is also an obstacle for the robot. But be careful, the robot you are following will not be your obstacle unless it approaches the robot.<br>
(4)The controller must be dynamic, which means that, while executing one stage, it can jump any stage depending on the global input before executing any stage.<br>

Config files (.cfg) and (.world) must be downloaded into the Stage/worlds path.<br>

