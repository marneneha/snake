# Snake robot
This repository implements snake motion gaites
## Kinematics
- DH Parameter
![]()
- Transformation matrix
![]()
## Dynamic
- Gait Parameter
![Gait_snake_image](https://github.com/marneneha/snake/blob/master/Gait.png)
- Euation of motion

## Prerequisite
- OS: Unbuntu(20.04)
- ROS(Neotic)
- Gazebo(Inbuilt in Ubuntu)

## How to run
- Clone
   - Either clone the Repository in the catkin workspace using the command
   ```
   git clone https://github.com/marneneha/snake.git
   ```
   OR
   - Create a catkin workspace and clone the repository in the src folder using following command
   
   ``` 
    mkdir –p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/marneneha/snake_model.git
    ```
- Build and source
```
    cd ~/catkin_ws/
    catkin build snake
    source devel/setup.bash    
```
- Launch
```
roslaunch snake template_launch.launch
```
You should see following result
## Result
![Result_after_launch]()
## Linear Progression
```
rosrun snake LinearProgression.py
```
![LinearProgression]()

## LateralUndulation
```
rosrun snake LateralUndulation.py
``` 
![LateralUndulation]()

## Sidewinding
```
rosrun snake Sidewinding.py
```
![Sidewinding]()

## FKandIK
5. rosrun snake FKandIK.py