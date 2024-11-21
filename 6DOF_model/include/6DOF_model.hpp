#ifndef DYNAMICS_MATRICES_HPP
#define DYNAMICS_MATRICES_HPP

#include <Eigen/Dense>
#include <vector>
#include <rml/RML.h>
#include <qpOASES.hpp>


/**
 * @file dynamics_matrices.hpp
 * @brief A library for computing dynamics matrices and forces for underwater vehicles.
 * 
 * This library is based on Fossen's dynamic model for marine craft, which describes
 * the 6-DOF motion of a rigid body in water. The dynamic model accounts for the 
 * following components:
 * 
 * ## Dynamic Model Equation
 * The system dynamics are described by the following equation:
 * 
 * ```
 * tau = M * nuDot   // Inertial forces (acceleration effects)
 *     + C * nu      // Coriolis and centripetal forces
 *     + D * nu      // Hydrodynamic damping forces
 *     + g           // Restoring forces (gravity and buoyancy)
 * ```
 * 
 * Where:
 * - `tau`: Generalized forces vector (6-DOF forces and moments).
 * - `M`: Combined inertia matrix (rigid body and added mass effects).
 * - `nu`: Body-fixed velocity vector `[u, v, w, p, q, r]`.
 * - `nuDot`: Body-fixed acceleration vector.
 * - `C`: Coriolis and centripetal matrix (rigid body and added mass effects).
 * - `D`: Hydrodynamic damping matrix (linear and quadratic drag).
 * - `g`: Restoring forces and moments (gravity and buoyancy).
 * 
 * Additionally, `tau` can be expressed as the product of the allocation matrix (`W`) and 
 * the vector of thruster forces (`f`):
 * 
 * ```
 * tau = W * f
 * ```
 * 
 * The library provides functions to compute each of these matrices and their components,
 * as well as to optimize thruster forces to satisfy the dynamic model while minimizing
 * control effort.
 */



/**
 * @brief Compute the moment of inertia matrix (M).
 *        This matrix combines rigid body inertia and added mass effects.
 *        Equation: M = M_{RB} + M_{A}
 *        - M_{RB}: Rigid body inertia matrix (mass + rotational inertia).
 *        - M_{A}: Added mass matrix (from hydrodynamic effects).
 * @param mass Mass of the robot.
 * @param centerOfGravity Position of the center of gravity (CG) in the body frame.
 * @param inertiaMatrix Inertia tensor about the CG in the body frame.
 * @param addedMassDiagonal Diagonal terms of the added mass matrix.
 * @return Combined moment of inertia matrix.
 */
Eigen::Matrix<double, 6, 6> GetM(double mass, const Eigen::Vector3d& centerOfGravity, 
                                const Eigen::Matrix3d& inertiaMatrix, 
                                const Eigen::Matrix<double, 6, 1>& addedMassDiagonal);

/**
 * @brief Compute the Coriolis and centripetal matrix (C).
 *        This accounts for the effects of velocity-dependent forces and moments.
 *        Equation: C = C_{RB} + C_{A}
 *        - C_{RB}: Rigid body Coriolis matrix.
 *        - C_{A}: Added mass Coriolis matrix.
 * @param relativeVelocity 6-DOF velocity vector [u, v, w, p, q, r].
 * @param mass Mass of the robot.
 * @param centerOfGravity Position of the center of gravity (CG) in the body frame.
 * @param inertiaMatrix0 Inertia tensor about the CG in the body frame.
 * @return Combined Coriolis matrix.
 */
Eigen::Matrix<double, 6, 6> GetC(const Eigen::Matrix<double, 6, 1>& relativeVelocity, double mass, 
                                const Eigen::Vector3d& centerOfGravity, 
                                const Eigen::Matrix3d& inertiaMatrix0);

/**
 * @brief Compute the damping matrix (D).
 *        This accounts for hydrodynamic drag forces and moments.
 *        Equation: D = D_{L} + D_{Q}
 *        - D_{L}: Linear damping matrix.
 *        - D_{Q}: Quadratic damping matrix.
 * @param relativeVelocity 6-DOF velocity vector [u, v, w, p, q, r].
 * @param dampingDiagonal Diagonal damping coefficients (linear and quadratic).
 * @return Damping matrix.
 */
Eigen::Matrix<double, 6, 6> GetD(const Eigen::Matrix<double, 6, 1>& relativeVelocity, 
                                const Eigen::Matrix<double, 6, 1>& dampingDiagonal);

/**
 * @brief Compute the gravitational and buoyancy forces and moments (G).
 *        These forces arise from the weight and buoyancy acting on the robot.
 *        Equation: G = g(\eta)
 *        - Restoring forces depend on orientation and position.
 * @param pose 6-DOF pose vector [x, y, z, roll, pitch, yaw].
 * @param mass Mass of the robot.
 * @param buoyancyForce Buoyant force acting on the robot.
 * @param gravity Gravity vector in the inertial frame.
 * @param centerOfGravity Position of the center of gravity in the body frame.
 * @param centerOfBuoyancy Position of the center of buoyancy in the body frame.
 * @return Gravitational and buoyancy force vector.
 */
Eigen::Matrix<double, 6, 1> GetG(const Eigen::Matrix<double, 6, 1>& pose, double mass, 
                                double buoyancyForce, const Eigen::Vector3d& gravity, 
                                const Eigen::Vector3d& centerOfGravity, 
                                const Eigen::Vector3d& centerOfBuoyancy);

/**
 * @brief Generate the thrusters wrench matrix.
 *        This matrix maps individual thruster forces to the robot's 6-DOF forces and moments.
 *        Equation: τ = T F
 *        - T: Thruster wrench matrix.
 *        - F: Vector of individual thruster forces.
 * @param thrusterPositions Positions of thrusters relative to the robot's center of gravity.
 * @param thrusterOrientations Orientation of thrusters (Euler angles).
 * @return Thruster wrench matrix.
 */
Eigen::MatrixXd GetThrustersWrenchMatrix(const Eigen::MatrixXd& thrusterPositions, 
                                        const Eigen::MatrixXd& thrusterOrientations);

/**
 * @brief Compute the optimal forces for the system using optimization.
 *        This solves a quadratic programming problem to determine the optimal
 *        thruster forces that satisfy the dynamic model while minimizing control effort.
 *        
 *        Quadratic Programming (QP) Problem:
 *        Minimize: (1/2) F^T H F + f^T F
 *        Subject to:
 *        - A F = b  (Equality constraints from the dynamic model).
 *        - lb ≤ F ≤ ub (Inequality constraints, e.g., thruster limits).
 *        
 *        In this context:
 *        - \( A \): Thruster wrench matrix, mapping individual thruster forces to the total
 *                  forces and moments acting on the robot.
 *        - \( F \): Vector of thruster forces to be determined.
 *        - \( b \): Desired generalized force vector \( \tau \), derived from the dynamic model:
 *        
 *                  b = tau = M * nu_dot        // Inertial forces (acceleration effects)
 *                          + C * nu           // Coriolis and centripetal forces
 *                          + D * nu           // Hydrodynamic damping forces
 *                          + g;               // Restoring forces (gravity and buoyancy)
 *        
 *        - \( lb, ub \): Lower and upper bounds for the thruster forces.
 *        - \( H \): Weighting matrix to penalize large forces (reduces control effort).
 *        
 *        The QP solves for \( F \), ensuring that the computed forces satisfy
 *        the dynamics of the robot and any physical constraints on thruster output.
 * 
 * @param A Constraint matrix (thruster wrench matrix, maps forces to the 6-DOF wrench).
 * @param b Constraint vector (desired wrench vector \( \tau \) from the dynamic model).
 * @param upLowBounds Upper and lower bounds for the forces (lb ≤ F ≤ ub).
 * @param weights Diagonal weighting matrix (H) for the quadratic cost function.
 * @return Optimal force vector (F).
 */

Eigen::VectorXd GetForces(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b, 
                        const Eigen::MatrixXd& upLowBounds, const Eigen::VectorXd& weights);


/**
 * @brief Convert an Eigen matrix to a qpOASES array.
 * @tparam Derived Eigen matrix type.
 * @param m Eigen matrix.
 * @return Pointer to the qpOASES array.
 */

template <typename Derived>
qpOASES::real_t* ConvertEigenToQpOASESArray(const Eigen::MatrixBase<Derived>& m){
    int rows = m.rows();
    int cols = m.cols();
    qpOASES::real_t* B = new qpOASES::real_t[rows * cols];

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            B[i * cols + j] = static_cast<qpOASES::real_t>(m(i, j));
        }
    }
    return B;
}


/**
 * @brief Compute the actual acceleration (a).
 *        Uses the system matrix to calculate the response to applied forces.
 *        Equation: a = M⁻¹ τ
 *        - M: Inertia matrix.
 *        - τ: Generalized forces.
 * @param massMatrix Combined inertia matrix (M).
 * @param tau Generalized force vector.
 * @param acceleration Output acceleration vector.
 */
void GetAcceleration(const Eigen::MatrixXd& M, const Eigen::Matrix<double, 6, 1>& tau, 
                    Eigen::Matrix<double, 6, 1>& acceleration);



#endif // DYNAMICS_MATRICES_HPP
