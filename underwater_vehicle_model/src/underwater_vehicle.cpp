#include "underwater_vehicle_model/underwater_vehicle.hpp"
#include "rml/RML.h"

Underwater_Vehicle::Underwater_Vehicle() { }
//void print_matrix(const Eigen::MatrixXd& mat){
//    int r = mat.rows();
//    int c = mat.cols();
//    for()
//}

void Underwater_Vehicle::DirectDynamics(const Eigen::Vector6d& volt_bodyF, const Eigen::Vector6d& Fcable_bodyF,
                                        const Eigen::RotationMatrix& worldF_R_bodyF, const Eigen::Vector6d& linAngVel_,  Eigen::Vector6d& linAngAcc_)
{

    F_thruster = VoltageToForces(volt_bodyF);
    //F_cable = ComputeCableForce();
    //Eigen::Vector6d F_cable; F_cable.setZero();
    //F.setOnes();
    Eigen::RotationMatrix bodyF_R_worldF = worldF_R_bodyF.transpose();

    UpdateMatrices(linAngVel_, bodyF_R_worldF);

    //linAngAcc_ = Minv * (T * K * F - (C + D)*linAngVel_ - g);
    //linAngAcc_ = Minv * (T * K * F - (C + D)*linAngVel_ + g); correct one
    linAngAcc_ = Minv * (T * K * F_thruster - (C + D)*linAngVel_ + g_bodyF + Fcable_bodyF);
    //std::cout << "linAngAcc_ = "<< linAngAcc_ << std::endl;
    //std::cout << "linAngVel_ = "<< linAngVel_ << std::endl;
    //std::cout << "F_voltage = "<< F_thruster << std::endl;
    //std::cout << "T * K * F = "<< T * K * F_thruster<< std::endl;
    //std::cout << "C = "<< C << std::endl;
    //std::cout << "D = "<< D << std::endl;
    std::cout << "g = "<< g_bodyF << std::endl;

    //std::cout << "K = "<< K << std::endl;
    //std::cout << "T = "<< T << std::endl;
    //std::cout << "Minv = "<< Minv << std::endl;

    //Eigen::Matrix6d tem;
    //tem = T * K ;
    //std::cout << "tem = "<< tem<< std::endl;



    std::cout << std::endl;
}


Eigen::Matrix3d Underwater_Vehicle::get_TensorInertia()
{
    //Tensor Inertia
    /*
    double a = params.L/2;
    double b = params.H/2;
    double c = b;

    double Ix = (0.2) * params.m * (pow(b,2)+pow(c,2));
    double Iy = (0.2) * params.m * (pow(a,2)+pow(c,2));
    double Iz = (0.2) * params.m * (pow(a,2)+pow(b,2));
    */

    double Ix = params.Ixyz(0);
    double Iy = params.Ixyz(1);
    double Iz = params.Ixyz(2);

    Eigen::Matrix3d TensorInertia_g;
    Eigen::Matrix3d TensorInertia_b;

    //Inertia matrix about the body's center of gravity
    TensorInertia_g.setZero();
    TensorInertia_g << Ix, 0, 0,
        0, Iy, 0,
        0, 0, Iz;

    I0 = TensorInertia_g;
    //Inertia matrix about CB(center of bouyancy) coincident with the origin of the body frame.
    TensorInertia_b.setZero();
    TensorInertia_b = TensorInertia_g - params.m * (rml::Vect3ToSkew(params.CG) * rml::Vect3ToSkew(params.CG));

    return TensorInertia_b;
}

Eigen::Vector6d Underwater_Vehicle::VoltageToForces(const Eigen::Vector6d& volt){
    Eigen::Vector6d F;
    for(int i=0; i<6; i++)
        F[i] = -140.3 * pow(volt[i],9) + 389.9 * pow(volt[i],7) - 404.1 * pow(volt[i],5) + 176.0 * pow(volt[i],3) + 8.9 * volt[i];

    //std::cout << "F = "<< F << std::endl;

    return F;
}

Eigen::Vector6d Underwater_Vehicle::ComputeFcable_bodyF(const Eigen::Vector3d &s_pos_worldF, const Eigen::Vector3d &e_pos_worldF,
                                                        const float &length, const Eigen::RotationMatrix &bodyF_R_worldF){

    float k = Cable_params.stiffness;
    Eigen::Vector3d worldF_F, bodyF_F;
    Eigen::Vector3d worldF_M, bodyF_M;
    Eigen::Vector6d worldF_FandM, bodyF_FandM;
    Eigen::Vector3d delta_worldF = e_pos_worldF - s_pos_worldF; // the distance between the ends of cable

    //double length = GetCableCurrentLength();
    Eigen::Vector3d delta_x = delta_worldF * (1 -  length / delta_worldF.norm()); // the difference between the distance and the length of cable
    if (delta_worldF.norm() >= length){
        worldF_F = - k * delta_x;
        //worldF_F.x() = - worldF_F.x();
        //worldF_F.y() = - worldF_F.y();
        bodyF_F = bodyF_R_worldF * worldF_F;
        //bodyF_F.x() = bodyF_F.x();
        Eigen::Vector3d r;
        //bodyF_M = bodyF_F.cross(Cable_params.AttachPoint);
        bodyF_M = Cable_params.AttachPoint.cross(bodyF_F);
        bodyF_FandM << bodyF_F, bodyF_M;
    }
    else
        bodyF_FandM.setZero();
    worldF_M = bodyF_R_worldF.transpose() * bodyF_M;
    worldF_FandM << worldF_F, worldF_M;

    std::cout << "F_cable_bodyF = "<< bodyF_FandM << std::endl;
    std::cout << "F_cable_worldF = "<< worldF_FandM << std::endl;

    return bodyF_FandM;
}

Eigen::Matrix6d Underwater_Vehicle::getM()
{
    Eigen::Matrix6d M_RB;
    Eigen::Matrix3d I;
    Eigen::Matrix6d M;

    M_a.setZero();
    //M_a = -diagXYZKMN->asDiagonal().toDenseMatrix();
    //M_a = -params.M_a_diag.asDiagonal().toDenseMatrix(); original
    M_a.diagonal() = - params.M_a_diag; // but in conf the vector is negative so M_a is positive

    I = get_TensorInertia();

    M_RB.setZero();
    M_RB.block(0,0,3,3) = params.m * Eigen::MatrixXd::Identity(3,3);
    M_RB.block(0,3,3,3)= -params.m * rml::Vect3ToSkew(params.CG);
    M_RB.block(3,0,3,3)= params.m * rml::Vect3ToSkew(params.CG);
    M_RB.block(3,3,3,3) = I;

    M.setZero();
    M = M_RB + M_a;

    //std::cout << "M = "<< M << std::endl;

    return M;
}

Eigen::Matrix6d Underwater_Vehicle::getInvM()
{
    Eigen::Matrix6d M_inv; // inverse of the system inertia matrix
    rml::RegularizationData regData;
    regData.params.lambda = 0.001;
    regData.params.threshold = 0.00001;
    M_inv = rml::RegularizedPseudoInverse(M, regData);
    return M_inv;
}


Eigen::Matrix6d Underwater_Vehicle::getC(const Eigen::Vector6d &v_rel)
{
    Eigen::Vector3d v1, v2;
    v1 = v_rel.segment(0,3);
    v2 = v_rel.segment(3,3);

    Eigen::Vector3d d1, d2;
    d1 = params.M_a_diag.segment(0,3);
    d2 = params.M_a_diag.segment(3,3);

    Eigen::Matrix3d I;
    I = get_TensorInertia();

    Eigen::Matrix6d C_RB;
    C_RB.setZero();
    C_RB.block(0,0,3,3) = params.m * rml::Vect3ToSkew(v2);
    C_RB.block(0,3,3,3) = -(params.m * rml::Vect3ToSkew(v2)) * rml::Vect3ToSkew(params.CG);
    C_RB.block(3,0,3,3) = (params.m * rml::Vect3ToSkew(params.CG)) * rml::Vect3ToSkew(v2);
    C_RB.block(3,3,3,3) = -rml::Vect3ToSkew(I0*v2);

    Eigen::Matrix6d C_a;
    C_a.setZero(); v1.array();
    C_a.block(3,0,3,3) = rml::Vect3ToSkew(v1.array()*d1.array());
    C_a.block(0,3,3,3) = rml::Vect3ToSkew(v1.array()*d1.array());
    C_a.block(3,3,3,3) = rml::Vect3ToSkew(v2.array()*d2.array());
    /*
    C_a.block(3,0,3,3) = rml::Vect3ToSkew(v1.cwiseProduct(d1));
    C_a.block(0,3,3,3) = rml::Vect3ToSkew(v1.cwiseProduct(d1));
    C_a.block(3,3,3,3) = rml::Vect3ToSkew(v2.cwiseProduct(d2));
    */


    /* Eigen::VectorXf M_a_diagonal = -M_a.diagonal();
    C_a.setZero();
    C_a.block(0,0,3,3) = rml::Vect3ToSkew(v2);
    C_a.block(0,3,3,3) = Eigen::MatrixXf::Zero(3,3);
    C_a.block(3,0,3,3) = rml::Vect3ToSkew(v1);
    C_a.block(3,3,3,3) = rml::Vect3ToSkew(v2); */ /////

    Eigen::Matrix6d C;
    C = C_RB + C_a; // as says the definition

    //std::cout << "C = "<< C << std::endl;

    return C;
}

Eigen::Matrix6d Underwater_Vehicle::getD(const Eigen::Vector6d &v_rel)
{
    Eigen::Vector6d s;
    s = v_rel.cwiseAbs();

    Eigen::Vector6d x;
    //x = params.D_diag.cwiseProduct(v_rel);
    x.array() = params.D_diag.array() * s.array();

    Eigen::Matrix6d D;
    D.setZero();
    D.diagonal() = - x;
    //D.diagonal() = - v_rel.transpose()*params.D_diag.asDiagonal();

    //std::cout << "D = "<< D<< std::endl;

    return D;
}

Eigen::Vector6d Underwater_Vehicle::getg_bodyF(const Eigen::RotationMatrix& bodyF_R_worldF)
{
    //Eigen::Vector6d g_bodyF;
    g_bodyF.setZero();

    //float delta = m/rho;
    //float boyancy = delta * rho;

    //float phi = eta(3);
    //float theta = eta(4);
    //float psi = eta(5);

    /*Eigen::Matrix3d worldF_R_bodyF;
    Eigen::Matrix3d bodyF_R_worldF;
    worldF_R_bodyF = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX());
    bodyF_R_worldF = worldF_R_bodyF.transpose(); // from inertial frame to body frame (I->B)*/

    Eigen::Vector3d worldF_F; // forces with respect to earth frame
    Eigen::Vector3d bodyF_F; // forces with respect to body frame
    Eigen::Vector3d fG = params.m * params.G * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d fB = - params.B * Eigen::Vector3d::UnitZ();
    worldF_F = fG + fB;
    //std::cout << "fG = "<< fG << std::endl;
    //std::cout << "fB = "<< fB << std::endl;
    //std::cout << "params.m = "<< params.m << std::endl;
    //std::cout << "params.G = "<< params.G << std::endl;
    //std::cout << "params.B = "<< params.B << std::endl;
    bodyF_F = bodyF_R_worldF * worldF_F;

    Eigen::Vector3d worldF_M; // moments with respect to earth frame
    Eigen::Vector3d bodyF_M; // moments with respect to body frame
    //M_e = rml::Vect3ToSkew(params.CG) * fG + rml::Vect3ToSkew(params.CB) * fB;
    bodyF_M = params.CG.cross(bodyF_R_worldF * fG) + params.CB.cross(bodyF_R_worldF * fB);

    //bodyF_M = bodyF_R_worldF * worldF_M;
    /* Eigen::Vector3f k0, C_GB;
    k0 = Rt * Eigen::Vector3f::UnitZ();
    C_GB = CG - CB;
    g.segment(0,3) = -(m * G - B) * k0;
    g.segment(3,3) = -((m * G - B) * rml::Vect3ToSkew(CG) + m * G * rml::Vect3ToSkew(C_GB)) * k0;
    //std::cout << "g: " << g << std::endl; */////////////

    g_bodyF.segment(0,3) = bodyF_F;
    g_bodyF.segment(3,3) = bodyF_M;

    //std::cout << "g = "<< g<< std::endl;

    return g_bodyF;
}

float Underwater_Vehicle::GetCableCurrentLength(){
    return cable_length;
}

void Underwater_Vehicle::SetCableLength(const double &l){
    if(l <= Cable_params.length_full)
        cable_length =l;
}

void Underwater_Vehicle::InitializeMatrices(const Eigen::Vector6d &v_rel, const Eigen::RotationMatrix& bodyF_R_worldF){
    K = params.K_diag.asDiagonal();
    T.row(0) = params.T_vector.segment(0,6);
    T.row(1) = params.T_vector.segment(6,6);
    T.row(2) = params.T_vector.segment(12,6);
    T.row(3) = params.T_vector.segment(18,6);
    T.row(4) = params.T_vector.segment(24,6);
    T.row(5) = params.T_vector.segment(30,6);

    M = getM();
    Minv = getInvM();
    C = getC(v_rel);
    D = getD(v_rel);
    g_bodyF = getg_bodyF(bodyF_R_worldF);

    Cable_params.AttachPoint = {-params.L / 2, 0.0, 0.0};
}

void Underwater_Vehicle::UpdateMatrices(const Eigen::Vector6d &v_rel, const Eigen::RotationMatrix& bodyF_R_worldF){
    C = getC(v_rel);
    D = getD(v_rel);
    g_bodyF = getg_bodyF(bodyF_R_worldF);
}


