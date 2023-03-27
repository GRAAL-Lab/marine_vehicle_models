#include "underwater_vehicle_model/underwater_vehicle_model.hpp"
#include "rml/RML.h"

Underwater_Vehicle_Model::Underwater_Vehicle_Model() { }

void Underwater_Vehicle_Model::DirectDynamics(double h_p, double h_s, double& n_p, double& n_s, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_)
{

    /* Eigen::Vector3d nir, tauStar;
    nir.setZero();
    tauStar.setZero();

    Eigen::Vector3d tau = ComputeCoriolisAndDragForces(linAngVel);

    tauStar(0) = tau[0];
    tauStar(1) = tau[1];
    tauStar(2) = tau[2];

    double motorlinearXVel_p = linAngVel(0) + linAngVel(5) * params.d;
    double motorlinearXVel_s = linAngVel(0) - linAngVel(5) * params.d;

    //double n_p = PercentageToRPM(h_p);
    //double n_s = PercentageToRPM(h_s);

    // thruster dynamics
    double rpm_gain_p = (n_p < 0) ? params.rpmDynNegPerc : params.rpmDynPosPerc;
    double rpm_gain_s = (n_s < 0) ? params.rpmDynNegPerc : params.rpmDynPosPerc;
    n_p = params.rpmDynState * n_p + rpm_gain_p * h_p;
    n_s = params.rpmDynState * n_s + rpm_gain_s * h_s;

    double thrust_force_p = GetThrusterForce(n_p, motorlinearXVel_p);
    double thrust_force_s = GetThrusterForce(n_s, motorlinearXVel_s);

    double k_p = (n_p >= 0) ? params.k_pos : params.k_neg;
    double k_s = (n_s >= 0) ? params.k_pos : params.k_neg;

    Eigen::Vector3d tauC;
    tauC(0) = thrust_force_p + thrust_force_s;
    tauC(1) = k_p * thrust_force_p + k_s * thrust_force_s;
    tauC(2) = (params.d + params.l * k_p) * thrust_force_p + (-params.d + params.l * k_s) * thrust_force_s;

    rml::RegularizationData regData;
    regData.params.lambda = 0.001;
    regData.params.threshold = 0.00001;
    Eigen::Matrix3d I_pinv = rml::RegularizedPseudoInverse(params.Inertia, regData);
    nir = I_pinv * (tauC - tauStar);

    linAngAcc(0) = nir(0);
    linAngAcc(1) = nir(1);
    linAngAcc(2) = 0.0;
    linAngAcc(3) = 0.0;
    linAngAcc(4) = 0.0;
    linAngAcc(5) = nir(2); */
}

Eigen::Vector3d Underwater_Vehicle_Model::ComputeCoriolisAndDragForces(Eigen::Vector6d vehvel){

}

Eigen::Matrix3f Underwater_Vehicle_Model::get_TensorInertia()
{
    //Tensor Inertia
    double a = L/2;
    double b = H/2;
    double c = b;

    double Ix = (0.2) * m * (pow(b,2)+pow(c,2));
    double Iy = (0.2) * m * (pow(a,2)+pow(c,2));
    double Iz = (0.2) * m * (pow(a,2)+pow(b,2));

    Eigen::Matrix3f TensorInertia_g;
    Eigen::Matrix3f TensorInertia_b;

    //Inertia matrix about the body's center of gravity
    TensorInertia_g.setZero();
    TensorInertia_g << Ix, 0, 0,
        0, Iy, 0,
        0, 0, Iz;

    //Inertia matrix about CB(center of bouyancy) coincident with the origin of the body frame.
    TensorInertia_b.setZero();
    TensorInertia_b = TensorInertia_g - m * (rml::Vect3ToSkew(CG)*rml::Vect3ToSkew(CG));

    return TensorInertia_b;
}

Eigen::MatrixXf Underwater_Vehicle_Model::getM()
{
    Eigen::MatrixXf M_RB(6,6);
    Eigen::MatrixXf I(3,3);
    Eigen::MatrixXf M(6,6);

    M_a.setZero();
    //M_a = -diagXYZKMN->asDiagonal().toDenseMatrix();
    M_a = -M_a_diag->asDiagonal().toDenseMatrix();

    I = get_TensorInertia();

    M_RB.setZero();
    M_RB.block(0,0,3,3) = Eigen::MatrixXf::Identity(3,3)*m;
    M_RB.block(0,3,3,3)= -m * rml::Vect3ToSkew(CG);
    M_RB.block(3,0,3,3)= m * rml::Vect3ToSkew(CG);
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
    d1 = M_a_diag->segment(0,3);
    d2 = M_a_diag->segment(3,3);

    Eigen::Matrix3f I;
    I = get_TensorInertia();

    Eigen::MatrixXf C_RB(6,6);
    C_RB.setZero();
    C_RB.block(0,0,3,3) = m * rml::Vect3ToSkew(v2);
    C_RB.block(0,3,3,3) = -(m * rml::Vect3ToSkew(v2)) * rml::Vect3ToSkew(CG);
    C_RB.block(3,0,3,3) = (m * rml::Vect3ToSkew(CG)) * rml::Vect3ToSkew(v2);
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
    D.diagonal() = - v_rel.transpose()*D_diag->asDiagonal();
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
    Eigen::Vector3f fG = m * G * Eigen::Vector3f::UnitZ();
    Eigen::Vector3f fB = - B * Eigen::Vector3f::UnitZ();
    F_e = fG + fB;
    F_b = Rt * F_e;

    Eigen::Vector3f M_e; // moments with respect to earth frame
    Eigen::Vector3f M_b; // moments with respect to body frame
    M_e = CG * rml::Vect3ToSkew(fG) + CB * rml::Vect3ToSkew(fB);
    M_b = Rt * M_e;
    /* Eigen::Vector3f k0, C_GB;
    k0 = Rt * Eigen::Vector3f::UnitZ();
    C_GB = CG - CB;
    g.segment(0,3) = -(m * G - B) * k0;
    g.segment(3,3) = -((m * G - B) * rml::Vect3ToSkew(CG) + m * G * rml::Vect3ToSkew(C_GB)) * k0;
    //std::cout << "g: " << g << std::endl; */

    return g;
}

bool Underwater_Vehicle_Model::LoadConfiguration(const libconfig::Config& confObj) noexcept(false)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& blueROVmodel = root["blueROVmodel"];

    if (!ctb::GetParamVector(blueROVmodel, m, "m"))
        return false;

    /*if (!ctb::GetParamVector(ulisseModel, cN, "cN"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cX, "cX"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cY, "cY"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cNneg, "cNneg"))
            return false;
        if (!ctb::GetParam(ulisseModel, lambda_neg, "lambdaNeg"))
            return false;
        if (!ctb::GetParam(ulisseModel, lambda_pos, "lambdaPos"))
            return false;
        if (!ctb::GetParam(ulisseModel, b1_neg, "b1Neg"))
            return false;
        if (!ctb::GetParam(ulisseModel, b1_pos, "b1Pos"))
            return false;
        if (!ctb::GetParam(ulisseModel, b2_neg, "b2Neg"))
            return false;
        if (!ctb::GetParam(ulisseModel, b2_pos, "b2Pos"))
            return false;
        if (!ctb::GetParam(ulisseModel, d, "motorsTransversalDistance"))
            return false;
        if (!ctb::GetParam(ulisseModel, hullWidth, "hullWidth"))
            return false;
        if (!ctb::GetParam(ulisseModel, l, "motorsLongitudinalDistance"))
            return false;fG
        if (!ctb::GetParam(ulisseModel, k_pos, "kPos"))
            return false;
        if (!ctb::GetParam(ulisseModel, k_neg, "kNeg"))
            return false;
        if (!ctb::GetParam(ulisseModel, rpmDynState, "rpmDynState"))
            return false;
        if (!ctb::GetParam(ulisseModel, rpmDynPosPerc, "rpmDynPosPerc"))
            return false;
        if (!ctb::GetParam(ulisseModel, rpmDynNegPerc, "rpmDynNegPerc"))
            return false;

    Eigen::Vector3d tmp_Inerzia;
    tmp_Inerzia.setZero();
    if (!ctb::GetParamVector(ulisseModel, tmp_Inerzia, "inertia"))
        return false;

    Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_Inerzia.data());*/

        return true;
}
