#ifndef DYNAMICS_MODEL_HPP
#define DYNAMICS_MODEL_HPP

#include <Eigen/Dense>
#include <vector>
#include <libconfig.h++>
#include <rml/RML.h>

/**
 * @class DynamicsModel
 * @brief A class for computing dynamics matrices and forces for underwater vehicles.
 * 
 * This class encapsulates the dynamics of an underwater vehicle based on Fossen's dynamic model.
 * It provides methods to compute the dynamics matrices and the vehicle's acceleration given applied forces.
 */
class DynamicsModel {
public:
    /**
     * @brief Constructor that initializes the model parameters from a configuration file.
     * @param config The libconfig configuration object.
     * @param model_name The name of the model in the configuration file.
     */
    DynamicsModel(const libconfig::Config& config, const std::string& model_name);

    /**
     * @brief Updates the dynamics matrices based on the current state.
     * @param velocity The current 6-DOF velocity vector [u, v, w, p, q, r].
     * @param pose The current 6-DOF pose vector [x, y, z, roll, pitch, yaw].
     */
    void UpdateModel(const Eigen::Matrix<double, 6, 1>& velocity, const Eigen::Matrix<double, 6, 1>& pose);

    /**
     * @brief Computes the acceleration of the vehicle given the applied forces.
     * @param forces The vector of applied forces from thrusters.
     * @return The computed acceleration vector.
     */
    Eigen::Matrix<double, 6, 1> ComputeAcceleration(const Eigen::VectorXd& forces);
    
    /**
     * @brief Get the number of thrusters.
     * @return Number of thrusters.
     */
    std::size_t GetNumThrusters() const;

private:
    // Model parameters
    double mass_;
    double buoyancy_;
    Eigen::Vector3d centerOfGravity_;
    Eigen::Vector3d centerOfBuoyancy_;
    Eigen::Matrix3d inertiaTensor_;
    Eigen::Vector3d gravityVector_;
    Eigen::Matrix<double, 6, 1> addedMassDiagonal_;
    Eigen::Matrix<double, 6, 1> dampingCoefficients_;
    Eigen::MatrixXd thrusterPositions_;
    Eigen::MatrixXd thrusterOrientations_;
    Eigen::MatrixXd thrustersWrenchMatrix_;

    // Upper and lower bounds for thruster forces
    Eigen::VectorXd thrusterUpperLimits_;
    Eigen::VectorXd thrusterLowerLimits_;
    Eigen::VectorXd thrusterAllocationWeights_;

    // Dynamics matrices
    Eigen::Matrix<double, 6, 6> M_;
    Eigen::Matrix<double, 6, 6> C_;
    Eigen::Matrix<double, 6, 6> D_;
    Eigen::Matrix<double, 6, 1> G_;

    // Current velocity (stored for use in acceleration computation)
    Eigen::Matrix<double, 6, 1> velocity_;

    // Private methods
    void ComputeThrustersWrenchMatrix();
    Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& vec);
};

#endif // DYNAMICS_MODEL_HPP
