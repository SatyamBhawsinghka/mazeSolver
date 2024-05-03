# mazeSolver

## 1. Install turtlebot3 packages for your current ros-{DISTRO}

## 2. Clone this repository

## 3. Loading Maze into Gazebo (Using ros-noetic)

`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`

`rosrun gazebo_ros spawn_model -file ~/{PATH_to_repo}/model_files/maze_model/model.sdf -sdf -model maze`

## 5. Run the program

`cd ~/{PATH_to_repo}`

`catkin build` or `catkin_make`

`source devel/setup.bash`

`roslaunch maze mazeSolver.launch start_mazeSolver:=True`

`rosservice call /mazesolver "solver_flag: true"`
