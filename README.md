# Not Lab 7

## Setup

Make sure that Docker and VSCode are installed on your computer.

Additional VSCode extensions required are:

- Dev Containers - [link](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- Docker - [link](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)

Clone this repository to `enre467_ws` folder in the home directory

```bash
git clone -b armgits-playground https://github.com/armgits/467-playground.git enre467_ws
```

Open the repository folder in VSCode and **Reopen it in conatiner**. The container
shall build and eventually start. First time takes some time.

Once in the container, use the VSCode terminal to start a bash session
(Do set the default profile as bash too) and do the remaining setup.

```bash
.devcontainer/setup.sh
```

>A lot of the steps in setting up the workspace are automated using the above scipt.

## Lab 6 Steps - Simulation only

1. Run the following command to start Gazebo with the UR3e arm in it:

    ```bash
    roslaunch ur3e_setup ur3e_gazebo.launch z_height:=0.77
    ```

    `z_height` is the height at which the robot is spawned in Gazebo.

2. In a different terminal window, run the following command to start MoveIt functionality and RViz:

    ```bash
    roslaunch ur3e_moveit_mrc ur3e_moveit.launch sim:=true
    ```

    This also spawns a back wall obstacle that is needed for safety reasons. However, if the wall has to be removed, it can be done by adding `spawn_wall:=false` to the end of the command.

3. The `moveit_tutorial` package has sample code for performing three tasks: 1. Move the robot to a joint goal, 2. Move the robot to a pose goal and 3. Move the robot from one point to another in a cartesian path. You can refer to the `tutorial.cpp` in the `moveit_tutorial` package for the sample code. This sample code uses the helper functions from `moviet_wrapper` package. In a new terminal, run the following command to run this sample code:

    ``` bash
    rosrun moveit_tutorial tutorial
    ```

4. You will use these helper functions in your code to move your robot in square and circle trajectories. A package for this lab is provided to you and the name of this package is `ur3e_trajectory`. Add your code to the files `square.cpp` and `circle.cpp` for square and circle trajectories.

    Run the following command to run your code for square or circle trajectories:

    ``` bash
    rosrun ur3e_trajectory square
    ```

    >Note: Replace square with circle if you want to run your circle code.

5. You need to calculate the error between the trajectory followed by your robot and the desired trajectory. To do this, you have to record the end effector positions while your robot traces the trajectory. The `RecordPose.cpp` file contains the code to record end effector positions at the rate of 2 Hz. It starts recording poses when the boolean parameter `record_pose` turns true. You have to set the value of this parameter to true before executing the trajectory and set it to false after trajectory executions. Look at the end of `tutorial.cpp` file in the `moveit_tutorial` package for sample implementation. The boolean parameter `record_pose` needs to be loaded to parameter server and the `RecordPose.cpp` program will look for that parameter from the parameter server. Run the following command to load the parameter:

    ```bash
    roslaunch ur3e_trajectory load_params.launch
    ```

    Edit the string variable `out_path` in the `RecordPose.cpp` file to the destination where you want to save your end effector poses. After this is done, Run the following command at the same time you run your code for square or circle trajectory:

    ```bash
    rosrun ur3e_trajectory RecordPose
    ```

    You can use the generated csv file of the end effector poses to plot the followed trajectory against the ideal trajectory.

6. After you are done with your simulation. You can run your code on the real UR3e arm. Ask one of the Teaching Assistants to help you.

> - Copy the `ur3e_trajectory` package in the source folder to a USB drive
> - Go to a lab computer and clone the original "Lab_7" repo on it, paste into its souce directory
> - Follow the steps from the original repo's README and test the packge.

## Commands to run your code on real UR3e robot

Come on, this is irrelavent on a personal computer since there's no access to
RAL's network on which the robots are connected.
