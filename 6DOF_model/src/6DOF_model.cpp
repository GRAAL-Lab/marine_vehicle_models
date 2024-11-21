#include "6DOF_model.hpp"


// Moment of inertia matrix = moment of inertia for rigid body + added mass moment of inertia
auto GetM(double mass, const Eigen::Vector3d& centerOfGravity, const Eigen::Matrix3d& inertiaMatrix,
          const Eigen::Matrix<double, 6, 1>& addedMassDiagonal) -> Eigen::Matrix<double, 6, 6> {
    Eigen::Matrix<double, 6, 6> rigidBodyMassMatrix, addedMassMatrix, totalMassMatrix;

    // Adding mass matrix
    addedMassMatrix.setZero();
    addedMassMatrix.diagonal() = -addedMassDiagonal;

    // Inertia tensor
    Eigen::Matrix3d skewCenterOfGravity;
    skewCenterOfGravity << 0, -centerOfGravity.z(), centerOfGravity.y(),
                           centerOfGravity.z(), 0, -centerOfGravity.x(),
                           -centerOfGravity.y(), centerOfGravity.x(), 0;

    // Rigid body mass matrix
    rigidBodyMassMatrix.setZero();
    rigidBodyMassMatrix.block(0, 0, 3, 3) = mass * Eigen::Matrix3d::Identity();
    rigidBodyMassMatrix.block(0, 3, 3, 3) = -mass * skewCenterOfGravity;
    rigidBodyMassMatrix.block(3, 0, 3, 3) = mass * skewCenterOfGravity;
    rigidBodyMassMatrix.block(3, 3, 3, 3) = inertiaMatrix;

    // Total mass matrix
    totalMassMatrix = rigidBodyMassMatrix + addedMassMatrix;
    return totalMassMatrix;
}

// Coriolis and centripetal matrix = Coriolis and centripetal matrix for rigid body + added mass Coriolis and centripetal matrix
auto GetC(const Eigen::Matrix<double, 6, 1>& relativeVelocity, double mass, const Eigen::Vector3d& centerOfGravity,
                    const Eigen::Matrix3d& inertiaMatrix0) -> Eigen::Matrix<double, 6, 6> {
    Eigen::Vector3d angularVelocity = relativeVelocity.segment(3, 3);
    Eigen::Matrix<double, 6, 6> rigidBodyCoriolisMatrix, addedMassCoriolisMatrix, totalCoriolisMatrix;

    // Skew matrix for angular velocity
    Eigen::Matrix3d skewAngularVelocity;
    skewAngularVelocity << 0, -angularVelocity.z(), angularVelocity.y(),
                            angularVelocity.z(), 0, -angularVelocity.x(),
                            -angularVelocity.y(), angularVelocity.x(), 0;

    // Skew matrix for center of gravity
    Eigen::Matrix3d skewCenterOfGravity;
    skewCenterOfGravity << 0, -centerOfGravity.z(), centerOfGravity.y(),
                            centerOfGravity.z(), 0, -centerOfGravity.x(),
                            -centerOfGravity.y(), centerOfGravity.x(), 0;

    // Rigid body Coriolis and centripetal matrix
    rigidBodyCoriolisMatrix.setZero();
    rigidBodyCoriolisMatrix.block(0, 0, 3, 3) = mass * skewAngularVelocity;
    rigidBodyCoriolisMatrix.block(0, 3, 3, 3) = -(mass * skewAngularVelocity) * skewCenterOfGravity;
    rigidBodyCoriolisMatrix.block(3, 0, 3, 3) = (mass * skewCenterOfGravity) * skewAngularVelocity;

    // Correcting the assignment to use a skew-symmetric matrix from inertiaMatrix0 * angularVelocity
    Eigen::Vector3d inertiaVelocity = inertiaMatrix0 * angularVelocity;
    Eigen::Matrix3d skewInertiaVelocity;
    skewInertiaVelocity << 0, -inertiaVelocity.z(), inertiaVelocity.y(),
                            inertiaVelocity.z(), 0, -inertiaVelocity.x(),
                            -inertiaVelocity.y(), inertiaVelocity.x(), 0;
    rigidBodyCoriolisMatrix.block(3, 3, 3, 3) = -skewInertiaVelocity;

    // Added mass Coriolis and centripetal matrix (simplified for explanation)
    addedMassCoriolisMatrix.setZero();

    totalCoriolisMatrix = rigidBodyCoriolisMatrix + addedMassCoriolisMatrix;
    return totalCoriolisMatrix;
}

// Damping matrix = damping matrix for rigid body + added mass damping matrix
auto  GetD(const Eigen::Matrix<double, 6, 1>& relativeVelocity,
                    const Eigen::Matrix<double, 6, 1>& dampingDiagonal) -> Eigen::Matrix<double, 6, 6> {
    Eigen::Matrix<double, 6, 6> dampingMatrix;
    dampingMatrix.setZero();
    dampingMatrix.diagonal() = (dampingDiagonal.array() * relativeVelocity.cwiseAbs().array()).matrix();
    return dampingMatrix;
}

// Gravitational and buoyancy forces and moments
auto GetG(const Eigen::Matrix<double, 6, 1>& pose, double mass, double buoyancyForce,
                    const Eigen::Vector3d& gravity, const Eigen::Vector3d& centerOfGravity,
                    const Eigen::Vector3d& centerOfBuoyancy) -> Eigen::Matrix<double, 6, 1> {
    double roll = pose(3), pitch = pose(4), yaw = pose(5);

    // Convert Euler angles (roll, pitch, yaw) to rotation matrix
    Eigen::Matrix3d bodyF_R_worldF = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    Eigen::Vector3d gravitationalForce = mass * gravity;
    Eigen::Vector3d buoyancy = -buoyancyForce * Eigen::Vector3d::UnitZ();

    Eigen::Matrix3d worldF_R_bodyF = bodyF_R_worldF.transpose();
    Eigen::Vector3d worldF_force = gravitationalForce + buoyancy;
    Eigen::Vector3d bodyF_force = worldF_R_bodyF * worldF_force;
    Eigen::Vector3d bodyF_moment = centerOfGravity.cross(bodyF_R_worldF * gravitationalForce) +
                                    centerOfBuoyancy.cross(bodyF_R_worldF * buoyancy);

    Eigen::Matrix<double, 6, 1> result;
    result << bodyF_force, bodyF_moment;
    return result;
}

// Thrusters wrench matrix
auto GetThrustersWrenchMatrix(const Eigen::MatrixXd& thrusterPositions, const Eigen::MatrixXd& thrusterOrientations) -> Eigen::MatrixXd {
    auto eulerToRotationMatrix = [](const Eigen::Vector3d& euler) -> Eigen::Matrix3d {
        Eigen::Quaternion<double> q =
            Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
        return q.toRotationMatrix();
    };

    int numThrusters = thrusterPositions.rows();
    Eigen::MatrixXd wrenchMatrix(6, numThrusters);

    for (int i = 0; i < numThrusters; ++i) {
        Eigen::Matrix3d thrusterRotation = eulerToRotationMatrix(thrusterOrientations.row(i).transpose());
        Eigen::Vector3d baseDirection(1, 0, 0);
        Eigen::Vector3d unitVector = thrusterRotation * baseDirection.normalized();
        Eigen::Vector3d thrusterPosition = thrusterPositions.row(i);
        Eigen::Vector3d moment = thrusterPosition.cross(unitVector);
        wrenchMatrix.block<3, 1>(0, i) = unitVector; // Force component
        wrenchMatrix.block<3, 1>(3, i) = moment;     // Moment component
    }
    return wrenchMatrix;
}

// Compute the forces for a system using optimization
auto GetForces(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b, const Eigen::MatrixXd& upLowBounds, const Eigen::VectorXd& weights) -> Eigen::VectorXd {
    int numVars = A.cols();
    Eigen::MatrixXd H = weights.asDiagonal();
    Eigen::VectorXd f = Eigen::VectorXd::Zero(numVars);
    Eigen::VectorXd lb = upLowBounds.col(0);
    Eigen::VectorXd ub = upLowBounds.col(1);
    Eigen::VectorXd lbA = b.cast<double>();
    Eigen::VectorXd ubA = b.cast<double>();
    qpOASES::SQProblem problem(numVars, A.rows(), qpOASES::HST_POSDEF);

    qpOASES::Options options;
    options.enableRegularisation = qpOASES::BT_TRUE;
    options.terminationTolerance = 1e-6;
    options.boundTolerance = 1e-6;
    options.printLevel = qpOASES::PL_NONE;
    problem.setOptions(options);

    qpOASES::int_t nWSR = 10000;

    qpOASES::real_t* H_d = ConvertEigenToQpOASESArray(H);
    qpOASES::real_t* f_d = ConvertEigenToQpOASESArray(f);
    qpOASES::real_t* A_d = ConvertEigenToQpOASESArray(A);
    qpOASES::real_t* lb_d = ConvertEigenToQpOASESArray(lb);
    qpOASES::real_t* ub_d = ConvertEigenToQpOASESArray(ub);
    qpOASES::real_t* lbA_d = ConvertEigenToQpOASESArray(lbA);
    qpOASES::real_t* ubA_d = ConvertEigenToQpOASESArray(ubA);

    Eigen::VectorXd x(numVars);
    if (problem.init(H_d, f_d, A_d, lb_d, ub_d, lbA_d, ubA_d, nWSR) == qpOASES::SUCCESSFUL_RETURN) {
        problem.getPrimalSolution(x.data());
    } else {
        Eigen::MatrixXd A_copy = A;
        double* thrustersWrenchMatrixData = A_copy.data();
        int rows = A.rows();
        int cols = A.cols();
        double* TWPInv = new double[cols * rows];
        double thresholdTW = 1e-4;
        double lambdaTW = 1e-2;
        double prodTW;
        int flagTW;

        rml::GT_RegPinv(thrustersWrenchMatrixData, rows, cols, TWPInv, thresholdTW, lambdaTW, &prodTW, &flagTW);
        Eigen::Map<Eigen::MatrixXd> TWPInvMatrix(TWPInv, cols, rows);
        x = TWPInvMatrix * b;
        delete[] TWPInv;
    }

    delete[] H_d;
    delete[] f_d;
    delete[] A_d;
    delete[] lb_d;
    delete[] ub_d;
    delete[] lbA_d;
    delete[] ubA_d;

    double scaleDown = 1.0;
    for (int i = 0; i < numVars; ++i) {
        if (x[i] > ub[i]) {
            scaleDown = std::min(scaleDown, ub[i] / x[i]);
        }
        if (x[i] < lb[i]) {
            scaleDown = std::min(scaleDown, lb[i] / x[i]);
        }
    }

    if (scaleDown < 1.0) {
        x *= scaleDown;
    }

    for (int i = 0; i < numVars; ++i) {
        x[i] = std::max(lb[i], std::min(x[i], ub[i]));
    }
    return x;
}

// Compute the acceleration for a system
void GetAcceleration(const Eigen::MatrixXd& M, const Eigen::Matrix<double, 6, 1>& tau, Eigen::Matrix<double, 6, 1>& acceleration) {
    int rows = M.rows();
    int cols = M.cols();
    std::vector<double> M_copy(M.data(), M.data() + M.size());
    double* M_data = M_copy.data();
    double* JPInv = new double[cols * rows];
    double threshold = 1e-6;
    double lambda = 1e-4;
    double prod;
    int flag;

    rml::GT_RegPinv(M_data, rows, cols, JPInv, threshold, lambda, &prod, &flag);
    Eigen::Map<Eigen::MatrixXd> MInv(JPInv, cols, rows);
    acceleration = MInv * tau;
    delete[] JPInv;
}
