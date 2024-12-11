#ifndef DYNAMICS_MODEL_HPP
#define DYNAMICS_MODEL_HPP

#include <Eigen/Dense>
#include <vector>
#include <libconfig.h++>
#include <rml/RML.h>
#include <cmath>

namespace mvm {

/**
 * @class DynamicsModel
 * @brief A class for computing dynamics matrices and forces for underwater vehicles.
 * 
 * This class encapsulates the dynamics of an underwater vehicle based on Fossen's dynamic model.
 * It provides methods to compute the dynamics matrices, vehicle acceleration, and applied forces.
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
     * @brief Update the model based on current velocity and pose.
     * 
     * Updates the dynamics matrices (`M`, `C`, `D`, and `g`) based on the current velocity and pose.
     * @param velocity The current velocity vector (6-DOF).
     * @param pose The current pose vector (6-DOF).
     */
    void UpdateModel(const Eigen::Matrix<double, 6, 1>& velocity, const Eigen::Matrix<double, 6, 1>& pose);

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

    /**
     * @brief Get the mass matrix (`M`).
     * @return A constant reference to the mass matrix.
     */
    const Eigen::Matrix<double, 6, 6>& GetMassMatrix() const;

    /**
     * @brief Get the Coriolis and centripetal matrix (`C`).
     * @return A constant reference to the Coriolis matrix.
     */
    const Eigen::Matrix<double, 6, 6>& GetCoriolisMatrix() const;

    /**
     * @brief Get the hydrodynamic damping matrix (`D`).
     * @return A constant reference to the damping matrix.
     */
    const Eigen::Matrix<double, 6, 6>& GetDampingMatrix() const;

    /**
     * @brief Get the restoring forces and moments (`g`).
     * @return A constant reference to the restoring forces vector.
     */
    const Eigen::Matrix<double, 6, 1>& GetRestoringForces() const;

private:
    // Model parameters
    double mass_;                                  ///< Mass of the vehicle.
    double buoyancy_;                              ///< Buoyancy of the vehicle.
    Eigen::Vector3d centerOfGravity_;             ///< Center of gravity of the vehicle.
    Eigen::Vector3d centerOfBuoyancy_;            ///< Center of buoyancy of the vehicle.
    Eigen::Matrix3d inertiaTensor_;               ///< Inertia tensor of the vehicle.
    Eigen::Vector3d gravityVector_;               ///< Gravity vector.

    // Velocity
    Eigen::Matrix<double, 6, 1> velocity_;        ///< Velocity vector.

    Eigen::Matrix<double, 6, 1> addedMassDiagonal_; ///< Added mass diagonal matrix.
    Eigen::Matrix<double, 6, 1> dampingCoefficients_; ///< Damping coefficients.
    Eigen::MatrixXd thrusterPositions_;           ///< Positions of the thrusters.
    Eigen::MatrixXd thrusterOrientations_;        ///< Orientations of the thrusters.
    Eigen::MatrixXd thrustersWrenchMatrix_;       ///< Wrench matrix for thrusters.

    // Upper and lower bounds for thruster forces
    Eigen::VectorXd thrusterUpperLimits_;         ///< Upper force limits for thrusters.
    Eigen::VectorXd thrusterLowerLimits_;         ///< Lower force limits for thrusters.
    Eigen::VectorXd thrusterAllocationWeights_;   ///< Weights for thruster allocation.

    // Dynamics matrices
    Eigen::Matrix<double, 6, 6> massMatrix_;         ///< Mass matrix (`M`).
    Eigen::Matrix<double, 6, 6> coriolisMatrix_;     ///< Coriolis matrix (`C`).
    Eigen::Matrix<double, 6, 6> dampingMatrix_;      ///< Damping matrix (`D`).
    Eigen::Matrix<double, 6, 1> restoringForces_;    ///< Restoring forces (`g`).

    // Private helper methods
    /**
     * @brief Generate the thrusters wrench matrix (`W`).
     * Maps individual thruster forces to the vehicle's generalized forces and moments.
     */
    void ComputeThrustersWrenchMatrix();
};

} // namespace mvm

#endif // DYNAMICS_MODEL_HPP
