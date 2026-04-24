# Marine Vehicle Models

A collection of C++ libraries implementing dynamic models for marine vehicles.

---

## Surface Vehicle Model

The **SurfaceVehicleModel** library implements a 3-DOF planar dynamic model of a surface vehicle propelled by two thrusters (differential-drive / twin-screw configuration), expressed in N.E.D. (North–East–Down) coordinates.

### Key Features
- 3-DOF (surge, sway, yaw) dynamic modeling.
- Thruster force computation, including positive/negative quadrant models and RPM dynamics.
- Coriolis and drag force computation.
- Direct dynamics (acceleration from thruster commands) and inverse thruster allocation.
- Parameter configuration via `libconfig`.

### Prerequisites
- [Eigen](https://eigen.tuxfamily.org/) (linear algebra)
- [libconfig](https://hyperrealm.github.io/libconfig/) (configuration management)
- [RML Library](https://github.com/GRAAL-Lab/rml)
- [ctrl_toolbox](https://github.com/GRAAL-Lab/ctrl_toolbox)

### Building the Library
```bash
cd marine_vehicle_models/surface_vehicle_model
mkdir build && cd build
cmake ..
make
sudo make install
```

### Configuration File
The library reads vehicle parameters from a `libconfig` file (see `conf/ulisse_model.conf` for a complete example). Key parameters include inertia, thruster geometry, thruster force coefficients, and RPM dynamics coefficients.

---

## ROV Model

The **Rov** library implements a 6-DOF dynamic model of a tethered ROV (e.g. BlueROV), based on the Newton–Euler formulation. It supports both standard and heavy thruster configurations and optionally models the dynamics of the tether cable.

### Key Features
- 6-DOF dynamic modeling (surge, sway, heave, roll, pitch, yaw).
- Mass matrix, Coriolis matrix, damping matrix, and restoring forces.
- Thruster allocation supporting 6-thruster (standard) and 8-thruster (heavy) configurations.
- Tether cable force computation and cable-winch dynamics.
- Parameter configuration via `libconfig`.

### Prerequisites
- [Eigen](https://eigen.tuxfamily.org/) (linear algebra)
- [libconfig](https://hyperrealm.github.io/libconfig/) (configuration management)
- [RML Library](https://github.com/GRAAL-Lab/rml)
- [ctrl_toolbox](https://github.com/GRAAL-Lab/ctrl_toolbox)

### Building the Library
```bash
cd marine_vehicle_models/rov_model
mkdir build && cd build
cmake ..
make
sudo make install
```

### Configuration File
The library reads vehicle parameters from a `libconfig` file (see `conf/blueROV.conf` for a complete example). Key parameters include mass and buoyancy properties, added mass, damping coefficients, thruster allocation matrix, and cable/winch parameters.

---

## Underwater Vehicle Model

The **UnderwaterVehicleModel** library provides a comprehensive implementation of 6-DOF underwater vehicle dynamics, based on Fossen's dynamic model. It supports AUV and ROV configurations.

See the [underwater_vehicle_model README](underwater_vehicle_model/README.md) for full documentation including installation instructions, API overview, and configuration file format.

---

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.
