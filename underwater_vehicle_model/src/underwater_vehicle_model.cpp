#include "underwater_vehicle_model/underwater_vehicle_model.hpp"
#include "rml/RML.h"

Underwater_Vehicle_Model::Underwater_Vehicle_Model() { }

void Underwater_Vehicle_Model::DirectDynamics(const Eigen::Vector6d& linAngVel_, const Eigen::Vector6d& eta, Eigen::Vector6d& linAngAcc_)
{


    UpdateMatrices(linAngVel_, eta);
    Eigen::MatrixXf Minv(6,6); // inverse of the system inertia matrix
    rml::RegularizationData regData;
    regData.params.lambda = 0.001;
    regData.params.threshold = 0.00001;
    Minv = rml::RegularizedPseudoInverse(M, regData);
    linAngAcc_ = Minv * (params.B_motor * params.u_motor - (C + D)*linAngVel_ - g);
}

Eigen::Vector3d Underwater_Vehicle_Model::ComputeCoriolisAndDragForces(Eigen::Vector6d vehvel){

}

Eigen::Matrix3f Underwater_Vehicle_Model::get_TensorInertia()
{
    //Tensor Inertia
    double a = params.L/2;
    double b = params.H/2;
    double c = b;

    double Ix = (0.2) * params.m * (pow(b,2)+pow(c,2));
    double Iy = (0.2) * params.m * (pow(a,2)+pow(c,2));
    double Iz = (0.2) * params.m * (pow(a,2)+pow(b,2));

    Eigen::Matrix3f TensorInertia_g;
    Eigen::Matrix3f TensorInertia_b;

    //Inertia matrix about the body's center of gravity
    TensorInertia_g.setZero();
    TensorInertia_g << Ix, 0, 0,
        0, Iy, 0,
        0, 0, Iz;

    //Inertia matrix about CB(center of bouyancy) coincident with the origin of the body frame.
    TensorInertia_b.setZero();
    TensorInertia_b = TensorInertia_g - params.m * (rml::Vect3ToSkew(params.CG)*rml::Vect3ToSkew(params.CG));

    return TensorInertia_b;
}

Eigen::MatrixXf Underwater_Vehicle_Model::getM()
{
    Eigen::MatrixXf M_RB(6,6);
    Eigen::MatrixXf I(3,3);
    Eigen::MatrixXf M(6,6);

    M_a.setZero();
    //M_a = -diagXYZKMN->asDiagonal().toDenseMatrix();
    M_a = -params.M_a_diag->asDiagonal().toDenseMatrix();

    I = get_TensorInertia();

    M_RB.setZero();
    M_RB.block(0,0,3,3) = Eigen::MatrixXf::Identity(3,3)*params.m;
    M_RB.block(0,3,3,3)= -params.m * rml::Vect3ToSkew(params.CG);
    M_RB.block(3,0,3,3)= params.m * rml::Vect3ToSkew(params.CG);
    M_RB.block(3,3,3,3) = I;

    M.setZero();
    M = M_RB + M_a; // - or + ?????

    return M;
}

Eigen::MatrixXf Underwater_Vehicle_Model::getC(const Eigen::VectorXf &v_rel)
{
    Eigen::Vector3f v1, v2;
    v1 = v_rel.segment(0,3);
    v2 = v_rel.segment(3,3);

    Eigen::Vector3f d1, d2;
    d1 = params.M_a_diag->segment(0,3);
    d2 = params.M_a_diag->segment(3,3);

    Eigen::Matrix3f I;
    I = get_TensorInertia();

    Eigen::MatrixXf C_RB(6,6);
    C_RB.setZero();
    C_RB.block(0,0,3,3) = params.m * rml::Vect3ToSkew(v2);
    C_RB.block(0,3,3,3) = -(params.m * rml::Vect3ToSkew(v2)) * rml::Vect3ToSkew(params.CG);
    C_RB.block(3,0,3,3) = (params.m * rml::Vect3ToSkew(params.CG)) * rml::Vect3ToSkew(v2);
    C_RB.block(3,3,3,3) = -rml::Vect3ToSkew(I*v2);

    Eigen::MatrixXf C_a(6,6);
    C_a.setZero();
    C_a.block(3,0,3,3) = rml::Vect3ToSkew(d1.transpose()*v1);
    C_a.block(0,3,3,3) = rml::Vect3ToSkew(d1.transpose()*v1);
    C_a.block(3,3,3,3) = rml::Vect3ToSkew(d2.transpose()*v2);
    /* Eigen::VectorXf M_a_diagonal = -M_a.diagonal();
    C_a.setZero();
    C_a.block(0,0,3,3) = rml::Vect3ToSkew(v2);
    C_a.block(0,3,3,3) = Eigen::MatrixXf::Zero(3,3);
    C_a.block(3,0,3,3) = rml::Vect3ToSkew(v1);
    C_a.block(3,3,3,3) = rml::Vect3ToSkew(v2); */

    Eigen::MatrixXf C(6,6);
    C = C_RB + C_a;

    return C;
}

Eigen::MatrixXf Underwater_Vehicle_Model::getD(const Eigen::VectorXf &v_rel)
{
    Eigen::MatrixXf D(6,6);
    D.setZero();
    D.diagonal() = - v_rel.transpose()*params.D_diag->asDiagonal();
    return D;
}

Eigen::MatrixXf Underwater_Vehicle_Model::getg(const Eigen::VectorXf &eta)
{
    Eigen::VectorXf g(6);
    g.setZero();

    //float delta = m/rho;
    //float boyancy = delta * rho;

    float phi = eta(3);
    float theta = eta(4);
    float psi = eta(5);

    Eigen::Matrix3f R;
    Eigen::Matrix3f Rt;
    R = Eigen::AngleAxisf(psi, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitX());
    Rt = R.transpose(); // from inertial frame to body frame (I->B)

    Eigen::Vector3f F_e; // forces with respect to earth frame
    Eigen::Vector3f F_b; // forces with respect to body frame
    Eigen::Vector3f fG = params.m * params.G * Eigen::Vector3f::UnitZ();
    Eigen::Vector3f fB = - params.B * Eigen::Vector3f::UnitZ();
    F_e = fG + fB;
    F_b = Rt * F_e;

    Eigen::Vector3f M_e; // moments with respect to earth frame
    Eigen::Vector3f M_b; // moments with respect to body frame
    M_e = params.CG * rml::Vect3ToSkew(fG) + params.CB * rml::Vect3ToSkew(fB);
    M_b = Rt * M_e;
    /* Eigen::Vector3f k0, C_GB;
    k0 = Rt * Eigen::Vector3f::UnitZ();
    C_GB = CG - CB;
    g.segment(0,3) = -(m * G - B) * k0;
    g.segment(3,3) = -((m * G - B) * rml::Vect3ToSkew(CG) + m * G * rml::Vect3ToSkew(C_GB)) * k0;
    //std::cout << "g: " << g << std::endl; */

    g.segment(0,3) = F_b;
    g.segment(3,3) = M_b;

    return g;
}

void Underwater_Vehicle_Model::InitializeMatrices(const Eigen::VectorXf &v_rel,const Eigen::VectorXf &eta){
    M = getM();
    C = getC(v_rel);
    D = getD(v_rel);
    g = getg(eta);
}

void Underwater_Vehicle_Model::UpdateMatrices(const Eigen::VectorXf &v_rel,const Eigen::VectorXf &eta){
    C = getC(v_rel);
    D = getD(v_rel);
    g = getg(eta);
}

