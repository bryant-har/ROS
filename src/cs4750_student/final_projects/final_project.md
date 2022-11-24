# Final Project
## Before You Start
This final project is dependent on HW1 - HW5. We provide you with the solution for HW1- HW3. Please start this homework after you finish HW4 and HW5. Remember to source `dependencies_ws` and `finalhomework_ws` in every terminal. If you see `mushr_sim` not found error, source `dependencies_ws`.

To organize your workspace, 
```
cd ~
cp -r homework_ws/src/cs4750_student/hw4_planning finalhomework_ws/src/cs4750_student/hw4_planning
cd finalhomework_ws
catkin build
```

## \# Overview
Congratulations on completing previous assignments! The purpose of this "project" is to give you an idea about how previous parts you have implemented come together. In this homework, we have two projects for you to complete. Each of them contains the component of state estimation, planning, and control. We want to demonstrate how these components work together in arms and cars. In general, the planner takes a goal pose or configuration from state estimation methods and then generates a path that will be used by the controller later. The controller executes the path. We will go into the project details and show you how this pipeline works for both the robot arm and the car.


## \# Arm Final Project (30 points)

It is the year 2050, and robots are widely deployed in dangerous scenarios to replace human workers.
In the airport, there is a table conveyer belt, a table, and a trash bin. 
The robot should be programmed to pick up the items including a tray, a can, and a wallet on the conveyer belt and put them in certain places. First, it should pick up the tray and drop it on the table with a known target pose for pick-up and drop. Then, it should detect the can and wallet with particle filter before pick-up and drop them in the tray on the table and in the trash bin respectively with a known target pose.

In this system, the particle filter or the predefined target pose gives the target for the robot to go to. The planner then computes a path from start to goal. Since the target pose is in cartesian space, while the planner works in joint space, you need to use inverse kinematics to do the conversion. After this, you need to send the path to the controller, which executes the trajectory.

## Task
To complete this, you need to:
1. Call `compute_ik()` function to convert from cartesian space to joint space. See `arm_final_project/scripts/airport`, in function `plan_and_execute`.
2. Obtain the detected value from the particle filter for position of the can and wallet for picking up. See `arm_final_project/scripts/airport`, in function `main`.
3. Insert start and goal joint configuration to the roadmap of the planner. See `arm_final_project/scripts/airport`, in function `plan_and_execute`.
4. Execute the path with controller. See `arm_final_project/scripts/airport`, in function `plan_and_execute`.
5. In the main function, call `plan_and_execute` to perform the following tasks. See `arm_final_project/scripts/airport`, in function `main`:
    + Pick up the white tray
    + Drop it off on the table
    + Pick up the can when the detected position is within a given threshold
    + Throw the can into the trash
    + Pick up the wallet when the detected position is within a given threshold
    + Drop the wallet on the tray

## Execute Your Code
Run your code with 
```
# In terminal 1
roslaunch arm_final_project start.launch

# In terminal 2
rosrun arm_final_project airport
```

## Submit Your Code
For each task, please take a screenshot of your robot *right at the time* when it is executing the actions of pick-up and drop, and put them in `images` folder. You can see the robot pausing for a short period of time when doing pick-up and drop, during which you can take the screenshot. Name the files accordingly, e.g., pickup_tray.png. We will run the code on our end to ensure the robot works.

## Rubric
- pickup_tray, drop_tray 10
- pickup_can, drop_can 10
- pickup_wallet, drop_wallet 10


## \# Car Final Project: Rescue Cathy (70 points)
It is the year 2050, and autonomous ground vehicles (AGVs) are now used in hospitals. Cathy is a dying patient present in the emergency room. All the doctors are busy, so no one can come to rescue Cathy within the next 200 seconds. Tragically, she will die without medicine in 100 seconds. You are driving the AGV to get the medicine and bring it to Cathy. 

The AGV uses a particle filter for localization, and there's a planner to plan the path. There are 3 rooms in the hospital, i.e. room A, room B, and room C. In room A, there is a medicine that can extend her lifetime by 50 seconds. In room B, there is a button to call the doctor and let them come earlier by 50 seconds, which means they will come in 150 seconds. In room C, there is a medicine that can add 20 seconds to the patient's life. The AGV can get medicine if it reaches the center of the rooms. 

 Cathy is considered alive if her lifetime is greater than the time for doctor to come, and the AGV goes back to Cathy before the doctor comes. The AGV can only go back to Cathy once and can not move again after going back to her.

## Task
1. Launch the particle filter, planner, and controller in a launch file. In `launch/start_bot.launch`, launch the `particle_filter` node in `localization` package from `hw3_state_estimation/car_state_estimation`, launch the `planner` node from `car_final_project`, launch the `controller` node from `hw5_control/car_controller`.
2. Send the path from the planner to the controller. See `src/car_final_project/planner_ros.py`, in function `send_path`.
3. Implement the `is_goal_reached` in `scripts/surveillance_bot` to check if the AGV reaches the goal pose.
4. Determine the sequence of room for you to grab medicine in `sequence.json`. Each room is represented by a tuple with three elements, `[x, y, heading_angle]`. Feel free to play with the heading angles, or add waypoints in the middle for a better path. The `[x,y]` positions for the rooms and Cathy are: 
- A: [14.4, 3.69]
- B: [1.8, 5.4]
- C: [3.19, 1.1]
- Cathy: [9.7, 1.2]

Remember you need to go back to Cathy for these medicines to take effect.

The json file might look like:
```
{
    "sequence":[
        [14.4, 3.69, 1.57],
        [1.8, 5.4, 1.57],
        [3.19, 1.1, 0.0],
        [9.7, 1.2, 1.57]
    ]
}
```
## Execute Your Code
After you're done, you should be able to perform the surveillance by running
```
roslaunch car_final_project start_bot.launch map:='$(find cs4750)/maps/problem-setup.yaml' initial_x:=9.7 initial_y:=1.2 initial_theta:=1.57
```

Try to extend Cathy's lifetime as much as possible. Feel free to play around with any of the components involved. Do not add any additional dependency packages/libraries. Ensure that you are able to run the `start_bot.launch` with the changes you made. 


After the surveillance is complete, you should see an `output.txt` file under `submission`. In this file, there is a result of whether Cathy is dead or able to wait until the doctor comes. There will also be Cathy's lifetime, time taken, and the efficiency for extending Cathy's lifetime in this file. The lifetime and time are in seconds. We will create a leaderboard and grade it based on the efficiency for extending Cathy's life, which is lifetime/time. You can only join the leader board if Cathy is alive at the end of the day.

## Trouble Shooting
1. If you see the `AttributeError: 'NoneType' object has no attribute 'astype'` error, try closing the rviz window and run the program again, or adjuting the waypoints by inserting some waypoints in the sequence.
2. If you see `mushr_sim` not found error, source `dependencies_ws`.

## Submit Your Code
Make sure you use your Student ID (SID) as the leaderboard name when submitting.

## Rubric
10 points - Graded based on the leaderboard

30 points - Cathy is alive

30 points - Output file is correct
