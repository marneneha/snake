![](https://github.com/marneneha/snake/actions/workflows/python-package.yml/badge.svg)
# Snake robot
This repository implements snake motion gaits



## Kinematics
- DH Parameter

![DH_Parameter](https://github.com/marneneha/snake/blob/master/ResultImages/DH_Parameter_snake.png)
- Transformation matrix

Transformation Matrix from Base to End Effector in home position

![Transformation Matrix](https://github.com/marneneha/snake/blob/master/ResultImages/Transformation_Matrix.png)




## Dynamic
- Gait Parameter

**Tune these values to achieve different motions**

![Gait_snake_image](https://github.com/marneneha/snake/blob/master/ResultImages/Gait.png)
- Euation of motion

![Euation of motion](https://github.com/marneneha/snake/blob/master/ResultImages/Equation_Information.png)




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
![Result_after_launch](https://github.com/marneneha/snake/blob/master/ResultImages/Result_launch%20_of_snake_model_repo.png)
## Snake Motions
- Linear Progression
```
rosrun snake LinearProgression.py
```

![LinearProgression](https://github.com/marneneha/snake/blob/master/ResultImages/LinearPropagation.gif)

- LateralUndulation
```
rosrun snake LateralUndulation.py
``` 

![LateralUndulation](https://github.com/marneneha/snake/blob/master/ResultImages/LateralUndualtion.gif)

- Sidewinding
```
rosrun snake Sidewinding.py
```

![Sidewinding](https://github.com/marneneha/snake/blob/master/ResultImages/SideWinding.gif)

*Reference: ReBiS – Reconfigurable Bipedal Snake Robot*

*Credits: Rohan Thakker, Ajinkya Kamat, Sachin Bharambe, Shital Chiddarwar, and K. M. Bhurchandi*
