# UnderwaterVehicleModel Library

## Overview
The **UnderwaterVehicleModel** library provides a comprehensive implementation of underwater vehicle dynamics, based on Fossen's dynamic model. This library supports the computation of dynamics matrices, applied forces, and vehicle acceleration, encapsulating the complexities of underwater dynamics in a user-friendly API.

### Key Features
- Calculation of mass matrix, Coriolis matrix, damping matrix, and restoring forces.
- Support for 6-DOF (degrees of freedom) dynamic modeling.
- Parameter configuration via `libconfig`.

## Installation

### Prerequisites
To build and use the `UnderwaterVehicleModel` library, ensure the following dependencies are installed:

- [Eigen](https://eigen.tuxfamily.org/) (for linear algebra computations)
- [libconfig](https://hyperrealm.github.io/libconfig/) (for configuration management)
- [RML Library](https://github.com/GRAAL-Lab/rml)

### Building the Library
   ```bash
   git clone https://github.com/GRAAL-Lab/marine_vehicle_models
   cd marine_vehicle_models/underwater_vehicle_model
   mkdir build && cd build
   cmake ..
   make
   sudo make install
   ```

## Usage


### Configuration File
The library requires a configuration file (`x300.conf`) to initialize vehicle parameters. Below is an example:

```cfg
x300:
{
    mass: 30.0; #kg
    buoyancy: -294.3;
    center_of_gravity: [0.0, 0.0, 0.0]; # x, y, z coordinates
    inertia_tensor: [12.523398, 12.523398, 0.360375]; # kg*m^2
    gravity_vector: [0.0, 0.0, -9.81]; # m/s^2
    center_of_buoyancy: [0.0, 0.0, 0.0]; # x, y, z coordinates
    added_mass: [-9.6, -18.2, -18.2, 0.0, 218.261, 218.261]; # kg
    damping_coefficients_linear:[1.0,175.0,175.0,0.1,1.0,1.0];# Ns/m
    damping_coefficients_quadratic:[1.7,90.0,90.0,1.4,15.1,15.1];# Ns/m
    
    # thruster positions and orientations
    thruster_positions: [-1.0, 0.0, 0.0, -0.5, 0.0, 0.0, -0.25, 0.0, 0.0, 0.25, 0.0, 0.0, 0.5, 0.0, 0.0]; # x, y, z coordinates
    thruster_orientations_degrees: [0.0, 0.0, -180.0, 0.0, 0.0, 90.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 0.0, 90.0]; # x, y, z angles in degree

};
```

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.
