#include "underwater_vehicle_model/underwater_vehicle.hpp"
#include "rml/RML.h"

Underwater_Vehicle::Underwater_Vehicle() { }

void Underwater_Vehicle::DirectDynamics(const Eigen::Vector6d& volt,const Eigen::Vector6d& linAngVel_, const Eigen::Vector6d& eta, Eigen::Vector6d& linAngAcc_)
{

    F = VoltageToForces(volt);
    UpdateMatrices(linAngVel_, eta);
    Eigen::MatrixXd Minv(6,6); // inverse of the system inertia matrix
    rml::RegularizationData regData;
    regData.params.lambda = 0.001;
    regData.params.threshold = 0.00001;
    Minv = rml::RegularizedPseudoInverse(M, regData);
    linAngAcc_ = Minv * (T * K * F - (C + D)*linAngVel_ - g);
}


Eigen::Matrix3d Underwater_Vehicle::get_TensorInertia()
{
    //Tensor Inertia
    double a = params.L/2;
    double b = params.H/2;
    double c = b;

    double Ix = (0.2) * params.m * (pow(b,2)+pow(c,2));
    double Iy = (0.2) * params.m * (pow(a,2)+pow(c,2));
    double Iz = (0.2) * params.m * (pow(a,2)+pow(b,2));

    Eigen::Matrix3d TensorInertia_g;
    Eigen::Matrix3d TensorInertia_b;

    //Inertia matrix about the body's center of gravity
    TensorInertia_g.setZero();
    TensorInertia_g << Ix, 0, 0,
        0, Iy, 0,
        0, 0, Iz;

    //Inertia matrix about CB(center of bouyancy) coincident with the origin of the body frame.
    TensorInertia_b.setZero();
    //TensorInertia_b = TensorInertia_g - params.m * (rml::Vect3ToSkew(params.CG) * rml::Vect3ToSkew(params.CG));

    return TensorInertia_b;
}

Eigen::Vector6d Underwater_Vehicle::VoltageToForces(const Eigen::Vector6d& volt){
    Eigen::Vector6d F;
    for(int i=0; i<6; i++)
        F[i] = -140.3 * pow(volt[i],9) + 389.9 * pow(volt[i],7) - 404.1 * pow(volt[i],5) + 176.0 * pow(volt[i],3) + 8.9 * volt[i];
    return F;
}

Eigen::MatrixXd Underwater_Vehicle::getM()
{
    Eigen::MatrixXd M_RB(6,6);
    Eigen::MatrixXd I(3,3);
    Eigen::MatrixXd M(6,6);

    M_a.setZero();
    //M_a = -diagXYZKMN->asDiagonal().toDenseMatrix();
    M_a = -params.M_a_diag.asDiagonal().toDenseMatrix();

    I = get_TensorInertia();

    M_RB.setZero();
    M_RB.block(0,0,3,3) = params.m * Eigen::MatrixXd::Identity(3,3);
    M_RB.block(0,3,3,3)= -params.m * rml::Vect3ToSkew(params.CG);
    M_RB.block(3,0,3,3)= params.m * rml::Vect3ToSkew(params.CG);
    M_RB.block(3,3,3,3) = I;

    M.setZero();
    M = M_RB + M_a;

    return M;
}


Eigen::MatrixXd Underwater_Vehicle::getC(const Eigen::Vector6d &v_rel)
{
    Eigen::Vector3d v1, v2;
    v1 = v_rel.segment(0,3);
    v2 = v_rel.segment(3,3);

    Eigen::Vector3d d1, d2;
    d1 = params.M_a_diag.segment(0,3);
    d2 = params.M_a_diag.segment(3,3);

    Eigen::Matrix3d I;
    I = get_TensorInertia();

    Eigen::MatrixXd C_RB(6,6);
    C_RB.setZero();
    C_RB.block(0,0,3,3) = params.m * rml::Vect3ToSkew(v2);
    C_RB.block(0,3,3,3) = -(params.m * rml::Vect3ToSkew(v2)) * rml::Vect3ToSkew(params.CG);
    C_RB.block(3,0,3,3) = (params.m * rml::Vect3ToSkew(params.CG)) * rml::Vect3ToSkew(v2);
    C_RB.block(3,3,3,3) = -rml::Vect3ToSkew(I*v2);

    Eigen::MatrixXd C_a(6,6);
    C_a.setZero();
    C_a.block(3,0,3,3) = rml::Vect3ToSkew(v1.cwiseProduct(d1));
    C_a.block(0,3,3,3) = rml::Vect3ToSkew(v1.cwiseProduct(d1));
    C_a.block(3,3,3,3) = rml::Vect3ToSkew(v2.cwiseProduct(d2));

    /* Eigen::VectorXf M_a_diagonal = -M_a.diagonal();
    C_a.setZero();
    C_a.block(0,0,3,3) = rml::Vect3ToSkew(v2);
    C_a.block(0,3,3,3) = Eigen::MatrixXf::Zero(3,3);
    C_a.block(3,0,3,3) = rml::Vect3ToSkew(v1);
    C_a.block(3,3,3,3) = rml::Vect3ToSkew(v2); */ /////

    Eigen::MatrixXd C(6,6);
    C = C_RB + C_a;

    return C;
}

Eigen::MatrixXd Underwater_Vehicle::getD(const Eigen::Vector6d &v_rel)
{
    Eigen::MatrixXd D(6,6);
    D.setZero();
    D.diagonal() = - v_rel.transpose()*params.D_diag.asDiagonal();
    return D;
}

Eigen::Vector6d Underwater_Vehicle::getg(const Eigen::Vector6d &eta)
{
    Eigen::Vector6d g;
    g.setZero();

    //float delta = m/rho;
    //float boyancy = delta * rho;

    float phi = eta(3);
    float theta = eta(4);
    float psi = eta(5);

    Eigen::Matrix3d R;
    Eigen::Matrix3d Rt;
    R = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX());
    Rt = R.transpose(); // from inertial frame to body frame (I->B)

    Eigen::Vector3d F_e; // forces with respect to earth frame
    Eigen::Vector3d F_b; // forces with respect to body frame
    Eigen::Vector3d fG = params.m * params.G * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d fB = - params.B * Eigen::Vector3d::UnitZ();
    F_e = fG + fB;
    F_b = Rt * F_e;

    Eigen::Vector3d M_e; // moments with respect to earth frame
    Eigen::Vector3d M_b; // moments with respect to body frame
    //M_e = rml::Vect3ToSkew(params.CG) * fG + rml::Vect3ToSkew(params.CB) * fB;
    M_e = params.CG.cross(fG) + params.CB.cross(fB);
    M_b = Rt * M_e;
    /* Eigen::Vector3f k0, C_GB;
    k0 = Rt * Eigen::Vector3f::UnitZ();
    C_GB = CG - CB;
    g.segment(0,3) = -(m * G - B) * k0;
    g.segment(3,3) = -((m * G - B) * rml::Vect3ToSkew(CG) + m * G * rml::Vect3ToSkew(C_GB)) * k0;
    //std::cout << "g: " << g << std::endl; */////////////

    g.segment(0,3) = F_b;
    g.segment(3,3) = M_b;

    return g;
}

void Underwater_Vehicle::InitializeMatrices(const Eigen::Vector6d &v_rel,const Eigen::Vector6d &eta){
    K = params.K_diag.asDiagonal();
    T.row(0) = params.T_vector.segment(0,6);
    T.row(1) = params.T_vector.segment(6,6);
    T.row(2) = params.T_vector.segment(12,6);
    T.row(3) = params.T_vector.segment(18,6);
    T.row(4) = params.T_vector.segment(24,6);
    T.row(5) = params.T_vector.segment(30,6);

    M = getM();
    C = getC(v_rel);
    D = getD(v_rel);
    g = getg(eta);
}

void Underwater_Vehicle::UpdateMatrices(const Eigen::Vector6d &v_rel,const Eigen::Vector6d &eta){
    C = getC(v_rel);
    D = getD(v_rel);
    g = getg(eta);
}


