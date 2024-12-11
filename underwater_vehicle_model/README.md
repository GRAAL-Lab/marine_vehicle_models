# 6DOF_model Library

TO DO

## Dependencies

To build and use the `6DOF_model` library, the following dependencies must be installed on your system:

- **qpOASES**: An open-source C++ implementation of the online active set strategy for quadratic programming (QP).
- **RML**

## Installation Instructions

### Install qpOASES
The 6DOF_model library depends on the shared library version of qpOASES. Follow these steps to install qpOASES from its GitHub repository.

```bash
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make
sudo make install
```