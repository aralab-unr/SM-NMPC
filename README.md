# SM-NMPC: Sliding Mode-Based Nonlinear Model Predictive Control for UAVs under Motor Failure on Microcontrollers
# Problem formulation
We are considering the NMPC with the following formulation:
<img src="figures/formu1.png" width="1000">

With the reference as the input and the virtual control law proposed by the AHSMC, more details about the sub-controller can be found in the following manuscript:
[SM_NMPC_RA_L.pdf](https://github.com/user-attachments/files/19696574/SM_NMPC_RA_L.pdf)

# Code Generation
The code generation is based on ACADO code generation and the qpOASES solver; therefore, the following libraries are required:
- ACADO Toolkit (https://acado.github.io/)
- qpOASES (https://github.com/coin-or/qpOASES)

In order to generate the code of the NMPC for the Quadrotor UAVs, follow these commands:
```shell
# Step 1: Clone the repository and download the ACADO code generation folder
$ Download the acadogenquad folder from the main branch.

# Step 2: Navigate to the ACADO workspace and generate the code
$ cd acadogenquad/build
$ cmake ..
$ make

# Step 3: Run the code generation
$ cd ./acadogenquad
```

In order to generate the code of the NMPC for the Cube-Drone, follow these commands:
```shell
# Step 1: Clone the repository and download the ACADO code generation folder
$ Download the acadogencube folder from the main branch.

# Step 2: Navigate to the ACADO workspace and generate the code
$ cd acadogencube/build
$ cmake ..
$ make

# Step 3: Run the code generation
$ cd ./acadogencube
```

Now, the generated code is located in the acadogen_quad or the cube_gen folder, which includes the necessary include directory required to run the NMPC. For more details on code generation, the author strongly recommends visiting: [ACADO Getting Started Guide](https://docs.ros.org/en/kinetic/api/acado/html/sim_getting_started.html)

# Simulation

## Software Requirements & Setup

The simulation is configured with the following setup:
- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11
- Xarco-ROS-Humble (sudo apt install ros-humble-xacro)
- Gazebo_ros_pkgs (sudo apt install ros-humble-gazebo-ros-pkgs)
- ACADO Toolkit (https://acado.github.io/)


Follow these commands in order to install the simulation of SM-NMPC for the UAVs on ROS 2:

```shell
# Step 1: Create and build a colcon workspace:
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/
$ colcon build
$ echo "source ~/dev_ws/devel/setup.bash" >> ~/.bashrc

# Step 2: Clone this repo into your workspace
$ cd ~/dev_ws/src
Download the folder smcmpcquad or the smcnmpccube in the main branch

# Step 3: Build the colcon workspace for this package
$ cd ~/dev_ws
$ colcon build
```
* Note that the package contains the code generation and includes the qpOASES library. If the user wants to use SM-NMPC for a different problem, they need to regenerate the code and replace it to the include folder.
* Note that this project uses a custom plugin. Users need to replace the plugin path in the file /urdf/uav_drone.urdf.xacro at line 268. Replace: <plugin name="uavplugin" filename="/home/vanchung/dev_ws/install/smcmpcquad/lib/smcmpcquad/libuavplugin.so"> with the correct path by changing the username to the name of your computer. For the Cube-Drone, Replace line 1009 in the file /urdf/cube.urdf.xacro: <plugin name="cubeplugin" filename="/home/vanchung/dev_ws/install/smcnmpccube/lib/smcnmpccube/libcubeplugin.so"> with the correct path by changing the username to the name of your computer. Then rebuild the project again to run the simulation.


To run the NMPCM simulation, follow these commands:
```shell
# Step 1: Run the Gazebo model:
$ ros2 launch nmpcpidquad model.launch.py

# Step 2: Run the controller
$ ros2 run nmpcpidquad nmpcpidquad %for the nmpcm
or run
$ ros2 run nmpcpidquad cascadedpid %for the cascaded pid
```
To run the NMPC based on CasADi, follow these commands:

```shell
# Step 1: Run the Gazebo model:
$ ros2 launch nmpccasadiquad model.launch.py

# Step 2: Run the controller
$ ros2 run nmpccasadiquad nmpccasadiquad 
```


# Contact
- [Van Chung Nguyen](mailto:vanchungn@.unr.edu)
- [Hung La](mailto:hla@unr.edu)
