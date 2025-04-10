# SM-NMPC: Sliding Mode-Based Nonlinear Model Predictive Control for UAVs under Motor Failure on Microcontrollers

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

Now, the generated code is located in the acadogen_quad or the cube_gen folder, which includes the necessary include directory required to run NMPCM. For more details on code generation, the author strongly recommends visiting: [ACADO Getting Started Guide](https://docs.ros.org/en/kinetic/api/acado/html/sim_getting_started.html)

# Contact
- [Van Chung Nguyen](mailto:vanchungn@.unr.edu)
- [Hung La](mailto:hla@unr.edu)
