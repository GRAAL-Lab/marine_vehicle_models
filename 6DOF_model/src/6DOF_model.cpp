#include "6DOF_model.hpp"
#include <iostream>
#include <cmath>

DynamicsModel::DynamicsModel(const libconfig::Config& config, const std::string& model_name) {
    // Load parameters from the configuration object
    const libconfig::Setting& root = config.getRoot();

    try {
        const libconfig::Setting& model = root.lookup(model_name);

        // Load mass
        model.lookupValue("mass", mass_);

        // Load buoyancy
        model.lookupValue("buoyancy", buoyancy_);

        // Load center of gravity
        const libconfig::Setting& cog = model["center_of_gravity"];
        centerOfGravity_ = Eigen::Vector3d(cog[0], cog[1], cog[2]);

        // Load inertia tensor
        const libconfig::Setting& inertia = model["inertia_tensor"];
        inertiaTensor_ = Eigen::Matrix3d::Zero();
        inertiaTensor_(0, 0) = inertia[0];
        inertiaTensor_(1, 1) = inertia[1];
        inertiaTensor_(2, 2) = inertia[2];

        // Load gravity vector
        const libconfig::Setting& gravity = model["gravity_vector"];
        gravityVector_ = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]);

        // Load center of buoyancy
        const libconfig::Setting& cob = model["center_of_buoyancy"];
        centerOfBuoyancy_ = Eigen::Vector3d(cob[0], cob[1], cob[2]);

        // Load added mass
        const libconfig::Setting& addedMass = model["added_mass"];
        addedMassDiagonal_ = Eigen::Matrix<double, 6, 1>::Zero();
        for (int i = 0; i < 6; ++i) {
            addedMassDiagonal_(i) = addedMass[i];
        }

        // Load damping coefficients
        const libconfig::Setting& dampingCoefficients = model["damping_coefficients"];
        dampingCoefficients_ = Eigen::Matrix<double, 6, 1>::Zero();
        for (int i = 0; i < 6; ++i) {
            dampingCoefficients_(i) = dampingCoefficients[i];
        }

        // Load thruster upper and lower limits
        const libconfig::Setting& upperLimits = model["thruster_upper_limits"];
        const libconfig::Setting& lowerLimits = model["thruster_lower_limits"];
        int numThrusters = upperLimits.getLength();
        thrusterUpperLimits_ = Eigen::VectorXd(numThrusters);
        thrusterLowerLimits_ = Eigen::VectorXd(numThrusters);
        for (int i = 0; i < numThrusters; ++i) {
            thrusterUpperLimits_(i) = upperLimits[i];
            thrusterLowerLimits_(i) = lowerLimits[i];
        }

        // Load thruster allocation weights
        const libconfig::Setting& allocationWeights = model["thruster_allocation_weights"];
        thrusterAllocationWeights_ = Eigen::VectorXd(numThrusters);
        for (int i = 0; i < numThrusters; ++i) {
            thrusterAllocationWeights_(i) = allocationWeights[i];
        }

        // Load thruster positions
        const libconfig::Setting& positions = model["thruster_positions"];
        int numPositions = positions.getLength();
        thrusterPositions_.resize(numPositions / 3, 3);
        for (int i = 0; i < numPositions / 3; ++i) {
            thrusterPositions_(i, 0) = positions[i * 3];
            thrusterPositions_(i, 1) = positions[i * 3 + 1];
            thrusterPositions_(i, 2) = positions[i * 3 + 2];
        }

        // Load thruster orientations
        const libconfig::Setting& orientations = model["thruster_orientations_degrees"];
        int numOrientations = orientations.getLength();
        thrusterOrientations_.resize(numOrientations / 3, 3);
        for (int i = 0; i < numOrientations / 3; ++i) {
            double angle0 = orientations[i * 3];
            double angle1 = orientations[i * 3 + 1];
            double angle2 = orientations[i * 3 + 2];

            thrusterOrientations_(i, 0) = angle0 * M_PI / 180.0; // Convert to radians
            thrusterOrientations_(i, 1) = angle1 * M_PI / 180.0;
            thrusterOrientations_(i, 2) = angle2 * M_PI / 180.0;
        }

        // Compute the thrusters wrench matrix
        ComputeThrustersWrenchMatrix();

    } catch (const libconfig::SettingNotFoundException& nfex) {
        std::cerr << "Setting not found: " << nfex.getPath() << std::endl;
        throw;
    } catch (const libconfig::SettingTypeException& stex) {
        std::cerr << "Setting has wrong type: " << stex.getPath() << std::endl;
        throw;
    }
}

void DynamicsModel::ComputeThrustersWrenchMatrix() {
    // Number of thrusters
    int numThrusters = thrusterPositions_.rows();

    // Initialize the wrench matrix
    thrustersWrenchMatrix_.resize(6, numThrusters);

    for (int i = 0; i < numThrusters; ++i) {
        // For each thruster
        // Get the orientation
        Eigen::Vector3d orientation = thrusterOrientations_.row(i);

        // Convert Euler angles to rotation matrix
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix = (Eigen::AngleAxisd(orientation[2], Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(orientation[1], Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(orientation[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

        // Assume the thrust direction is along the x-axis in the thruster's frame
        Eigen::Vector3d baseDirection(1, 0, 0);

        // Compute the unit vector of thrust in the vehicle frame
        Eigen::Vector3d unitVector = rotationMatrix * baseDirection;

        // Get the thruster position
        Eigen::Vector3d thrusterPosition = thrusterPositions_.row(i);

        // Compute the moment arm
        Eigen::Vector3d moment = thrusterPosition.cross(unitVector);

        // Set the wrench matrix
        thrustersWrenchMatrix_.block<3, 1>(0, i) = unitVector; // Force component
        thrustersWrenchMatrix_.block<3, 1>(3, i) = moment;     // Moment component
    }
}

void DynamicsModel::UpdateModel(const Eigen::Matrix<double, 6, 1>& velocity, const Eigen::Matrix<double, 6, 1>& pose) {
    velocity_ = velocity;

    // Compute M_ = M_RB + M_A
    // Rigid body mass matrix M_RB
    Eigen::Matrix<double, 6, 6> M_RB;
    M_RB.setZero();

    // Mass
    M_RB.block<3, 3>(0, 0) = mass_ * Eigen::Matrix3d::Identity();

    // Skew-symmetric matrix of center of gravity
    Eigen::Matrix3d S_r_G = SkewSymmetric(centerOfGravity_);

    // Off-diagonal blocks
    M_RB.block<3, 3>(0, 3) = -mass_ * S_r_G;
    M_RB.block<3, 3>(3, 0) = mass_ * S_r_G;

    // Inertia tensor (assuming about center of gravity)
    M_RB.block<3, 3>(3, 3) = inertiaTensor_;

    // Added mass matrix M_A (diagonal)
    Eigen::Matrix<double, 6, 6> M_A = Eigen::Matrix<double, 6, 6>::Zero();
    M_A.diagonal() = -addedMassDiagonal_;

    // Total mass matrix
    M_ = M_RB + M_A;

    // Compute C_
    // Extract velocities
    Eigen::Vector3d linearVelocity = velocity.segment<3>(0);
    Eigen::Vector3d angularVelocity = velocity.segment<3>(3);

    // Compute skew-symmetric matrices
    Eigen::Matrix3d S_omega = SkewSymmetric(angularVelocity);

    // Compute C_RB
    Eigen::Matrix<double, 6, 6> C_RB;
    C_RB.setZero();

    // Mass * skew(velocity)
    C_RB.block<3, 3>(0, 0) = mass_ * S_omega;

    // mass * skew(omega) * skew(r_G)
    C_RB.block<3, 3>(0, 3) = -mass_ * S_omega * S_r_G;

    // mass * skew(r_G) * skew(omega)
    C_RB.block<3, 3>(3, 0) = mass_ * S_r_G * S_omega;

    // Skew(I * omega)
    Eigen::Vector3d I_omega = inertiaTensor_ * angularVelocity;
    Eigen::Matrix3d S_Iomega = SkewSymmetric(I_omega);

    C_RB.block<3, 3>(3, 3) = -S_Iomega;

    // For the added mass Coriolis matrix C_A, we'll assume it's negligible
    // Total Coriolis matrix
    C_ = C_RB;

    // Compute D_
    D_.setZero();
    D_.diagonal() = (dampingCoefficients_.array() * velocity.array().abs()).matrix();

    // Compute G_
    // Extract orientation angles
    double roll = pose(3);
    double pitch = pose(4);
    double yaw = pose(5);

    // Compute rotation matrix from body to inertial frame
    Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    // Compute gravity force in inertial frame
    Eigen::Vector3d gravityForce = mass_ * gravityVector_;
    Eigen::Vector3d buoyancyForceVec = -buoyancy_ * Eigen::Vector3d::UnitZ();

    // Transform to body frame
    Eigen::Vector3d totalForceBody = R.transpose() * (gravityForce + buoyancyForceVec);

    // Compute moments due to gravity and buoyancy
    Eigen::Vector3d momentGravity = centerOfGravity_.cross(R.transpose() * gravityForce);
    Eigen::Vector3d momentBuoyancy = centerOfBuoyancy_.cross(R.transpose() * buoyancyForceVec);
    Eigen::Vector3d totalMomentBody = momentGravity + momentBuoyancy;

    // Assemble G_
    G_.segment<3>(0) = totalForceBody;
    G_.segment<3>(3) = totalMomentBody;
}

Eigen::Matrix<double, 6, 1> DynamicsModel::ComputeAcceleration(const Eigen::VectorXd& forces) {
    // Compute the net generalized forces
    Eigen::Matrix<double, 6, 1> tau = thrustersWrenchMatrix_ * forces;

    // Compute the right-hand side
    Eigen::Matrix<double, 6, 1> rhs = tau - (C_ * velocity_ + D_ * velocity_ + G_);

    // Solve for acceleration: M_ * acceleration = rhs
    Eigen::Matrix<double, 6, 1> acceleration = M_.ldlt().solve(rhs);

    return acceleration;
}

Eigen::Matrix3d DynamicsModel::SkewSymmetric(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skew;
    skew <<     0, -vec(2),  vec(1),
             vec(2),      0, -vec(0),
            -vec(1),  vec(0),      0;
    return skew;
}

std::size_t DynamicsModel::GetNumThrusters() const {
    return static_cast<std::size_t>(thrustersWrenchMatrix_.cols());
}