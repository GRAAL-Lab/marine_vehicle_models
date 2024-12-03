#ifndef DYNAMICS_MODEL_HPP
#define DYNAMICS_MODEL_HPP

#include <Eigen/Dense>
#include <vector>
#include <libconfig.h++>
#include <rml/RML.h>
#include <iostream>
#include <cmath>

/**
 * @class DynamicsModel
 * @brief A class for computing dynamics matrices and forces for underwater vehicles.
 * 
 * This class encapsulates the dynamics of an underwater vehicle based on Fossen's dynamic model.
 * It provides methods to compute the dynamics matrices, vehicle acceleration, and desired forces.
 * 
 * ## Dynamic Model Equation
 * The dynamics are described by:
 * ```
 * tau = M * nuDot   // Inertial forces (acceleration effects)
 *     + C * nu      // Coriolis and centripetal forces
 *     + D * nu      // Hydrodynamic damping forces
 *     + g           // Restoring forces (gravity and buoyancy)
 * ```
 * Where:
 * - `tau`: Generalized forces vector (6-DOF).
 * - `M`: Combined inertia matrix (rigid body + added mass effects).
 * - `nu`: Body-fixed velocity vector `[u, v, w, p, q, r]`.
 * - `nuDot`: Body-fixed acceleration vector.
 * - `C`: Coriolis and centripetal matrix.
 * - `D`: Hydrodynamic damping matrix.
 * - `g`: Restoring forces and moments.
 */
class DynamicsModel {
public:
    /**
     * @brief Constructor that initializes the model parameters from a configuration file.
     * 
     * Loads the parameters from the provided `libconfig` object for the specified model.
     * @param config The libconfig configuration object.
     * @param model_name The name of the model to load from the configuration.
     */
    DynamicsModel(const libconfig::Config& config, const std::string& model_name);

    /**
     * @brief Update the model based on actual velocity and pose (for simulation).
     * 
     * Updates the dynamics matrices (`M`, `C`, `D`, and `g`) based on the current velocity and pose.
     * @param velocity The current velocity vector (6-DOF).
     * @param pose The current pose vector (6-DOF).
     */

    void UpdateActualModel(const Eigen::Matrix<double, 6, 1>& velocityActual, const Eigen::Matrix<double, 6, 1>& poseActual);

    /**
     * @brief Compute the desired model components for control purposes.
     * 
     * Computes the dynamics matrices (`M`, `C`, `D`, and `g`) and returns the left-hand side
     * vector of the dynamic equation based on desired states.
     * @param accelerationDesired The desired acceleration vector (6-DOF).
     * @param velocityDesired The desired velocity vector (6-DOF).
     * @param poseDesired The desired pose vector (6-DOF).
     * @return The computed LHS vector based on desired dynamics.
     */
    Eigen::Matrix<double, 6, 1> ComputeDesiredModel(const Eigen::Matrix<double, 6, 1>& accelerationDesired,
                                                    const Eigen::Matrix<double, 6, 1>& velocityDesired,
                                                    const Eigen::Matrix<double, 6, 1>& poseDesired);

    /**
     * @brief Computes the acceleration of the vehicle given the applied forces.
     * 
     * Solves the equation: `M * acceleration = tau`, where `tau` includes
     * Coriolis, damping, and restoring forces, and returns the computed acceleration.
     * @param forces The vector of applied forces from thrusters.
     * @return The computed acceleration vector.
     */
    Eigen::Matrix<double, 6, 1> ComputeAcceleration(const Eigen::VectorXd& forces);

    /**
     * @brief Get the number of thrusters.
     * @return The number of thrusters.
     */
    std::size_t GetNumThrusters() const;

    /**
     * @brief Get the thrusters wrench matrix.
     * 
     * This matrix maps individual thruster forces to the robot's 6-DOF forces and moments:
     * ```
     * tau = W * f
     * ```
     * Where:
     * - `W`: Thrusters wrench matrix.
     * - `f`: Thruster forces.
     * @return A reference to the thrusters wrench matrix.
     */
    const Eigen::MatrixXd& GetThrustersWrenchMatrix() const;
    

private:
    // Model parameters
    double mass_;                                  ///< Mass of the vehicle.
    double buoyancy_;                              ///< Buoyancy of the vehicle.
    Eigen::Vector3d centerOfGravity_;             ///< Center of gravity of the vehicle.
    Eigen::Vector3d centerOfBuoyancy_;            ///< Center of buoyancy of the vehicle.
    Eigen::Matrix3d inertiaTensor_;               ///< Inertia tensor of the vehicle.
    Eigen::Vector3d gravityVector_;               ///< Gravity vector.

    // Velocity
    Eigen::Matrix<double, 6, 1> velocityActual_;  ///< Actual velocity vector.

    Eigen::Matrix<double, 6, 1> addedMassDiagonal_; ///< Added mass diagonal matrix.
    Eigen::Matrix<double, 6, 1> dampingCoefficients_; ///< Damping coefficients.
    Eigen::MatrixXd thrusterPositions_;           ///< Positions of the thrusters.
    Eigen::MatrixXd thrusterOrientations_;        ///< Orientations of the thrusters.
    Eigen::MatrixXd thrustersWrenchMatrix_;       ///< Wrench matrix for thrusters.

    // Upper and lower bounds for thruster forces
    Eigen::VectorXd thrusterUpperLimits_;         ///< Upper force limits for thrusters.
    Eigen::VectorXd thrusterLowerLimits_;         ///< Lower force limits for thrusters.
    Eigen::VectorXd thrusterAllocationWeights_;  ///< Weights for thruster allocation.

    // Dynamics matrices
    Eigen::Matrix<double, 6, 6> massMatrix_;         ///< Mass matrix (`M`).
    Eigen::Matrix<double, 6, 6> coriolisMatrix_;    ///< Coriolis matrix (`C`).
    Eigen::Matrix<double, 6, 6> dampingMatrix_;    ///< Damping matrix (`D`).
    Eigen::Matrix<double, 6, 1> restoringForces_;               ///< Restoring forces (`g`).

    // Private helper methods
    /**
     * @brief Compute the combined mass matrix (`M`).
     * Includes both rigid body inertia and added mass effects.
     */
    void UpdateMassMatrix();

    /**
     * @brief Compute the Coriolis and centripetal matrix (`C`).
     * Accounts for velocity-dependent forces and moments.
     * @param angularVelocity The angular velocity vector `[p, q, r]`.
     */
    void UpdateCoriolisMatrix(const Eigen::Vector3d& angularVelocity);

    /**
     * @brief Compute the hydrodynamic damping matrix (`D`).
     * Accounts for drag forces and moments.
     * @param velocity The body-fixed velocity vector.
     */
    void UpdateDampingMatrix(const Eigen::Matrix<double, 6, 1>& velocity);

    /**
     * @brief Compute the restoring forces and moments (`g`).
     * Accounts for gravity and buoyancy effects.
     * @param pose The body-fixed pose vector `[x, y, z, roll, pitch, yaw]`.
     */
    void UpdateGravityMatrix(const Eigen::Matrix<double, 6, 1>& pose);

    /**
     * @brief Generate the thrusters wrench matrix (`W`).
     * Maps individual thruster forces to the vehicle's generalized forces and moments.
     */
    void ComputeThrustersWrenchMatrix();

    /**
     * @brief Generate a skew-symmetric matrix for a given vector.
     * Used for cross-product operations in matrix form.
     * @param vec The input vector.
     * @return The skew-symmetric matrix.
     */
    Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& vec);
};

#endif // DYNAMICS_MODEL_HPP
