#include "6DOF_model.hpp"

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
    int numThrusters = thrusterPositions_.rows();
    if (numThrusters == 0) {
        std::cerr << "Error: Thruster positions are not set." << std::endl;
        return;
    }

    thrustersWrenchMatrix_.resize(6, numThrusters);
    for (int i = 0; i < numThrusters; ++i) {
        Eigen::Vector3d orientation = thrusterOrientations_.row(i);
        Eigen::Matrix3d rotationMatrix = (Eigen::AngleAxisd(orientation[2], Eigen::Vector3d::UnitZ())
                                         * Eigen::AngleAxisd(orientation[1], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(orientation[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Vector3d unitVector = rotationMatrix * Eigen::Vector3d::UnitX();
        Eigen::Vector3d position = thrusterPositions_.row(i).transpose();
        Eigen::Vector3d moment = position.cross(unitVector);
        thrustersWrenchMatrix_.block<3, 1>(0, i) = unitVector;
        thrustersWrenchMatrix_.block<3, 1>(3, i) = moment;
    }
}


void DynamicsModel::UpdateMassMatrix() {
    Eigen::Matrix<double, 6, 6> massMatrix_RB = Eigen::Matrix<double, 6, 6>::Zero();
    massMatrix_RB.block<3, 3>(0, 0) = mass_ * Eigen::Matrix3d::Identity();
    massMatrix_RB.block<3, 3>(0, 3) = -mass_ * SkewSymmetric(centerOfGravity_);
    massMatrix_RB.block<3, 3>(3, 3) = inertiaTensor_;
    massMatrix_ = massMatrix_RB - addedMassDiagonal_.asDiagonal().toDenseMatrix();

}

void DynamicsModel::UpdateCoriolisMatrix(const Eigen::Vector3d& angularVelocity) {
    Eigen::Matrix3d S_omega = SkewSymmetric(angularVelocity);
    Eigen::Matrix3d S_r_G = SkewSymmetric(centerOfGravity_);
    Eigen::Matrix<double, 6, 6> coriolisMatrix_RB = Eigen::Matrix<double, 6, 6>::Zero();
    coriolisMatrix_RB.block<3, 3>(0, 0) = mass_ * S_omega;
    coriolisMatrix_RB.block<3, 3>(0, 3) = -mass_ * S_omega * S_r_G;
    coriolisMatrix_RB.block<3, 3>(3, 3) = -SkewSymmetric(inertiaTensor_ * angularVelocity);
    coriolisMatrix_ = coriolisMatrix_RB;
}

void DynamicsModel::UpdateDampingMatrix(const Eigen::Matrix<double, 6, 1>& velocity) {
    dampingMatrix_ = dampingCoefficients_.cwiseProduct(velocity.cwiseAbs()).asDiagonal();
}

void DynamicsModel::UpdateGravityMatrix(const Eigen::Matrix<double, 6, 1>& pose) {
    Eigen::Matrix3d R = (Eigen::AngleAxisd(pose(5), Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(pose(4), Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(pose(3), Eigen::Vector3d::UnitX())).toRotationMatrix();
    Eigen::Vector3d gravityForce = mass_ * gravityVector_;
    Eigen::Vector3d buoyancyForce = -buoyancy_ * Eigen::Vector3d::UnitZ();
    restoringForces_.head<3>() = R.transpose() * (gravityForce + buoyancyForce);
    restoringForces_.tail<3>() = centerOfGravity_.cross(R.transpose() * gravityForce)
                 + centerOfBuoyancy_.cross(R.transpose() * buoyancyForce);
}




void DynamicsModel::UpdateActualModel(const Eigen::Matrix<double, 6, 1>& velocityActual, const Eigen::Matrix<double, 6, 1>& poseActual) {
    velocityActual_ = velocityActual;
    UpdateMassMatrix();
    UpdateCoriolisMatrix(velocityActual.tail<3>());
    UpdateDampingMatrix(velocityActual);
    UpdateGravityMatrix(poseActual);
}

Eigen::Matrix<double, 6, 1> DynamicsModel::ComputeDesiredModel(const Eigen::Matrix<double, 6, 1>& accelerationDesired,
                                       const Eigen::Matrix<double, 6, 1>& velocityDesired,
                                       const Eigen::Matrix<double, 6, 1>& poseDesired) {
    UpdateMassMatrix();
    UpdateCoriolisMatrix(velocityDesired.tail<3>());
    UpdateDampingMatrix(velocityDesired);
    UpdateGravityMatrix(poseDesired);
    UpdateActualModel(velocityDesired, poseDesired);
    return  (massMatrix_ * accelerationDesired + coriolisMatrix_ * velocityDesired + dampingMatrix_ * velocityDesired + restoringForces_);
}


Eigen::Matrix<double, 6, 1> DynamicsModel::ComputeAcceleration(const Eigen::VectorXd& forces) {
    // Compute the net generalized forces
    Eigen::Matrix<double, 6, 1> tau = thrustersWrenchMatrix_ * forces;

    // Compute the right-hand side
    Eigen::Matrix<double, 6, 1> rhs = tau - (coriolisMatrix_ * velocityActual_ + dampingMatrix_ * velocityActual_ + restoringForces_);

    // Solve for acceleration: massMatrix_ * acceleration = rhs
    Eigen::Matrix<double, 6, 1> acceleration = massMatrix_.ldlt().solve(rhs);

    return acceleration;
}

Eigen::Matrix3d DynamicsModel::SkewSymmetric(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skew;
    skew << 0, -vec.z(), vec.y(),
            vec.z(), 0, -vec.x(),
            -vec.y(), vec.x(), 0;
    return skew;
}

std::size_t DynamicsModel::GetNumThrusters() const {
    return thrustersWrenchMatrix_.cols();
}


const Eigen::MatrixXd& DynamicsModel::GetThrustersWrenchMatrix() const {
    return thrustersWrenchMatrix_;
}
