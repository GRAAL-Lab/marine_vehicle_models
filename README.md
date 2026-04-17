# Marine Vehicle Models

A collection of C++ libraries implementing dynamic models for marine vehicles, developed at the [GRAAL Laboratory – University of Genoa](https://graal.dibris.unige.it/).

## Repository Structure

```
marine_vehicle_models/
├── surface_vehicle_model/      # Surface vehicle (USV) with two thrusters – ROS 2 package
├── rov_model/                  # Tethered ROV (BlueROV) with cable model – ROS 2 package
└── underwater_vehicle_model/   # Generic AUV/ROV dynamic model – standalone C++ library
```

## Modules

### Surface Vehicle Model

Dynamic model of a surface vehicle operating in the NED (North-East-Down) frame.
The vehicle is equipped with two thrusters and the model captures hull drag, thruster dynamics (4-quadrant), and RPM dynamics.

- **ROS 2** package (`ament_cmake`)
- Configuration loaded from `conf/ulisse_model.conf`
- Key class: `SurfaceVehicleModel`
- See [`surface_vehicle_model/`](surface_vehicle_model/)

### ROV Model

Dynamic model of a tethered ROV based on the BlueROV platform.
Supports both the standard (6-thruster) and heavy (8-thruster) BlueROV configurations and includes a cable/winch model.

- **ROS 2** package (`ament_cmake`)
- Configuration loaded from `conf/blueROV.conf`
- Key class: `Rov`
- See [`rov_model/`](rov_model/)

### Underwater Vehicle Model

A standalone 6-DOF dynamic model for underwater vehicles (AUV or ROV) based on **Fossen's equations of motion**:

```
M * nuDot + C(nu) * nu + D(nu) * nu + g(eta) = tau
```

Where `M` is the combined inertia matrix (rigid body + added mass), `C` is the Coriolis/centripetal matrix, `D` is the hydrodynamic damping matrix, and `g` contains the restoring forces.

- Standalone C++ library (no ROS 2 required)
- Optional **Python bindings** via pybind11 (`BUILD_PYTHON_BINDINGS=ON`)
- Configuration loaded from a `libconfig` file (e.g. `conf/x300.conf`)
- Key class: `mvm::UnderwaterVehicleModel`
- See [`underwater_vehicle_model/`](underwater_vehicle_model/) for full build and usage instructions

## Dependencies

| Dependency | Surface Vehicle | ROV Model | Underwater Vehicle |
|---|:---:|:---:|:---:|
| [Eigen3](https://eigen.tuxfamily.org/) | ✓ | ✓ | ✓ |
| [libconfig++](https://hyperrealm.github.io/libconfig/) | ✓ | ✓ | ✓ |
| [RML](https://bitbucket.org/isme_robotics/rml/src/master/) | ✓ | ✓ | ✓ |
| [ROS 2](https://docs.ros.org/) | ✓ | ✓ | |
| [pybind11](https://pybind11.readthedocs.io/) | | | optional |

## Building

### ROS 2 packages (`surface_vehicle_model`, `rov_model`)

Place the desired package(s) inside a ROS 2 workspace and build with `colcon`:

```bash
cd ~/ros2_ws/src
# copy or symlink the package directory here
cd ~/ros2_ws
colcon build --packages-select surface_vehicle_model   # or rov_model
```

### Standalone library (`underwater_vehicle_model`)

```bash
cd underwater_vehicle_model
mkdir build && cd build
cmake ..
make
sudo make install
```

To disable the Python bindings:

```bash
cmake -DBUILD_PYTHON_BINDINGS=OFF ..
```

## License

Copyright © 2024 GRAAL Laboratory – University of Genoa.  
This project is licensed under the **MIT License**. See [LICENSE.md](LICENSE.md) for details.
