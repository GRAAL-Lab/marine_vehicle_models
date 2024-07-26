#include "underwater_vehicle_model/underwater_vehicle.hpp"
#include "rml/RML.h"
#include <cmath>

UnderwaterVehicle::UnderwaterVehicle() {
    /*if(!params.heavyConf){
        params.K_diag.conservativeResize(6,1);
        params.Q_diag.conservativeResize(6,1);
        K.resize(6,6);
        K = params.K_diag.asDiagonal();
        Q.resize(6,6);
        Q = params.Q_diag.asDiagonal();
        T.resize(6,6);
        T.row(0) = params.T_vector.segment(0,6);
        T.row(1) = params.T_vector.segment(6,6);
        T.row(2) = params.T_vector.segment(12,6);
        T.row(3) = params.T_vector.segment(18,6);
        T.row(4) = params.T_vector.segment(24,6);
        T.row(5) = params.T_vector.segment(30,6);
    }
    else{
        params.K_diag.conservativeResize(8,1);
        params.Q_diag.conservativeResize(8,1);
        K.resize(8,8);
        K = params.K_diag.asDiagonal();
        Q.resize(8,8);
        Q = params.Q_diag.asDiagonal();
        T.resize(6,8);
        T.row(0) = params.T_vector.segment(0,8);
        T.row(1) = params.T_vector.segment(8,8);
        T.row(2) = params.T_vector.segment(16,8);
        T.row(3) = params.T_vector.segment(24,8);
        T.row(4) = params.T_vector.segment(32,8);
        T.row(5) = params.T_vector.segment(40,8);
    }
    //params.Kp = K;*/
}

void UnderwaterVehicle::DirectDynamics(const Eigen::VectorXd& volt, const Eigen::Vector6d& bodyF_F_cable,
                                        const Eigen::RotationMatrix& worldF_R_bodyF, const Eigen::Vector6d& linAngVel_,  Eigen::Vector6d& linAngAcc_)
{
    //Eigen::Vector6d applied_volt;
    //F_th = VoltageToForces(volt);
    //F_cable = ComputeCableForce();
    //Eigen::Vector6d F_cable; F_cable.setZero();
    //F.setOnes();
    //Eigen::RotationMatrix bodyF_R_worldF = worldF_R_bodyF.transpose();

    UpdateMatrices(linAngVel_, worldF_R_bodyF);

    //linAngAcc_ = Minv * (T * K * F - (C + D)*linAngVel_ - g);
    //linAngAcc_ = Minv * (T * K * F - (C + D)*linAngVel_ + g); correct one
    //ThrustersSaturation(volt,1.0);
    linAngAcc_ = Minv * (params.T * params.K * params.Q * volt - (C + D)*linAngVel_ + bodyF_g + bodyF_F_cable);
    bodyF_F_coriolis_drag = - (C + D)*linAngVel_;
    body_F_thruster = params.T * params.K * params.Q * volt;
    //std::cout << "volt = "<< volt << std::endl;
    //std::cout << "T = "<< T << std::endl;
    //std::cout << "K = "<< K << std::endl;
    //std::cout << "body_F_thruster = T * K * volt = "<< body_F_thruster << std::endl;
    //std::cout << "linAngAcc_ = "<< linAngAcc_ << std::endl;
    //std::cout << "bodyF_F_cable = "<< bodyF_F_cable << std::endl;
    //std::cout << "linAngVel_ = "<< linAngVel_ << std::endl;
    //std::cout << "F_voltage = "<< body_F_thruster << std::endl;
    //std::cout << "T * K * F = "<< T * K * body_F_thruster<< std::endl;
    //std::cout << "C = "<< C << std::endl;
    //std::cout << "D = "<< D << std::endl;
    //std::cout << "g = "<< bodyF_g << std::endl;

    //std::cout << "K = "<< K << std::endl;
    //std::cout << "T = "<< T << std::endl;
    //std::cout << "Minv = "<< Minv << std::endl;

    //Eigen::Matrix6d tem;
    //tem = T * K ;
    //std::cout << "tem = "<< tem<< std::endl;



    //std::cout << std::endl;
}


Eigen::Matrix3d UnderwaterVehicle::get_TensorInertia()
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

Eigen::Vector6d UnderwaterVehicle::VoltageToForces(const Eigen::Vector6d& volt){
    Eigen::Vector6d F;
    for(int i=0; i<6; i++)
        F[i] = -140.3 * pow(volt[i],9) + 389.9 * pow(volt[i],7) - 404.1 * pow(volt[i],5) + 176.0 * pow(volt[i],3) + 8.9 * volt[i];

    //std::cout << "F = "<< F << std::endl;

    return F;
}


Eigen::Vector6d UnderwaterVehicle::ComputeFcable_bodyF(const Eigen::Vector3d &s_pos_worldF, const Eigen::Vector3d &e_pos_worldF,
                                                        const float &length, const Eigen::RotationMatrix &worldF_R_bodyF){

    Eigen::Vector3d worldF_F, bodyF_F;
    Eigen::Vector3d worldF_M, bodyF_M;
    Eigen::Vector6d worldF_FandM, bodyF_FandM;
    Eigen::Vector3d delta_worldF = e_pos_worldF - s_pos_worldF; // the distance between the ends of cable
    Eigen::RotationMatrix bodyF_R_worldF = worldF_R_bodyF.transpose();

    if (delta_worldF.norm() >= length - 0.5){
        //std::cout << "cableLength = "<< length << std::endl;
        Eigen::Vector3d v_cable = delta_worldF/delta_worldF.norm(); // unit vector, direction towards z+


        // Compute the projection of every force on cable vector
        // Check if the projection is positive otherwise it is not considered in cable force
        double Ftot = 0.0;
        double F;
        //Eigen::Vector6d FM_th = body_F_thruster; // Forces and Moments of thrusters in the body frame
        Eigen::Vector3d F_th = worldF_R_bodyF * body_F_thruster.head(3); // Forces of thrusters in the world frame
        F = F_th.dot(v_cable); // projection of thrusters forces on Cable vector
        if(F > 0) Ftot = Ftot + F; // if the force is positive (pulling the cable) take the force into the consideration


        Eigen::Vector3d F_g = worldF_R_bodyF * bodyF_g.head(3);
        F = F_g.dot(v_cable);
        if(F > 0) Ftot = Ftot + F;

        Eigen::Vector3d F_cd =  worldF_R_bodyF * bodyF_F_coriolis_drag.head(3);
        F = F_cd.dot(v_cable);
        if(F > 0) Ftot = Ftot + F;
        //std::cout << "F-cd = "<< F << std::endl;

        //double q;
        double r = delta_worldF.norm() - length;
        //if(r < 0) q = pow(2*r + 1, 2)*(1 - 4*r);
        //else q = 1;
        //worldF_F = - q * Ftot * v_cable;
        double rmin = - 0.5; double rmax = 0.0;
        double ymin = 0.0; double ymax = 1.0;
        double gain = rml::IncreasingBellShapedFunction(rmin, rmax, ymin, ymax, r);

        worldF_F = - gain * Ftot * v_cable;
        bodyF_F = bodyF_R_worldF * worldF_F;
        bodyF_M = Cable_params.AttachPoint.cross(bodyF_F);
        bodyF_FandM << bodyF_F, bodyF_M;
        worldF_M = worldF_R_bodyF * bodyF_M;
        worldF_FandM << worldF_F, worldF_M;
        //std::cout << "F_cable_bodyF = "<< bodyF_FandM << std::endl;
        //std::cout << "F_cable_worldF = "<< worldF_FandM << std::endl;
    }
    else
    {
        bodyF_FandM.setZero();
        worldF_FandM.setZero();
    }

    bodyF_F_cable = bodyF_FandM;
    return bodyF_FandM;
}

Eigen::Matrix6d UnderwaterVehicle::getM()
{
    Eigen::Matrix6d M_RB;
    Eigen::Matrix3d I;
    Eigen::Matrix6d M;

    M_a.setZero();
    M_a.diagonal() = - params.M_a_diag; // but in conf the vector is negative so M_a is positive

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

Eigen::Matrix6d UnderwaterVehicle::getInvM()
{
    Eigen::Matrix6d M_inv; // inverse of the system inertia matrix
    rml::RegularizationData regData;
    regData.params.lambda = 0.001;
    regData.params.threshold = 0.00001;
    M_inv = rml::RegularizedPseudoInverse(M, regData);
    return M_inv;
}


Eigen::Matrix6d UnderwaterVehicle::getC(const Eigen::Vector6d &v_rel)
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

    Eigen::Matrix6d C;
    C = C_RB + C_a; // as says the definition

    //std::cout << "C = "<< C << std::endl;

    return C;
}

Eigen::Matrix6d UnderwaterVehicle::getD(const Eigen::Vector6d &v_rel)
{
    Eigen::Vector6d s;
    s = v_rel.cwiseAbs();

    Eigen::Vector6d x;
    x.array() = params.D_diag.array() * s.array();

    Eigen::Matrix6d D;
    D.setZero();
    D.diagonal() = - x;

    //std::cout << "D = "<< D<< std::endl;

    return D;
}

Eigen::Vector6d UnderwaterVehicle::ComputeG_bodyF(const Eigen::RotationMatrix& worldF_R_bodyF)
{
    bodyF_g.setZero();

    Eigen::Vector3d worldF_F; // forces with respect to earth frame
    Eigen::Vector3d bodyF_F; // forces with respect to body frame
    Eigen::Vector3d fG = params.m * params.G * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d fB = - params.B * Eigen::Vector3d::UnitZ();
    worldF_F = fG + fB;
    Eigen::RotationMatrix bodyF_R_worldF = worldF_R_bodyF.transpose();
    bodyF_F = bodyF_R_worldF * worldF_F;

    Eigen::Vector3d worldF_M; // moments with respect to earth frame
    Eigen::Vector3d bodyF_M; // moments with respect to body frame
    //M_e = rml::Vect3ToSkew(params.CG) * fG + rml::Vect3ToSkew(params.CB) * fB;
    bodyF_M = params.CG.cross(bodyF_R_worldF * fG) + params.CB.cross(bodyF_R_worldF * fB);

    bodyF_g.segment(0,3) = bodyF_F;
    bodyF_g.segment(3,3) = bodyF_M;

    //std::cout << "g = "<< g<< std::endl;

    return bodyF_g;
}

float UnderwaterVehicle::GetCableReleasedLength(){
    return cable_length_released_;
}

float UnderwaterVehicle::GetCableLayer(){
    return current_layer;
}

float UnderwaterVehicle::GetCableWindingRadius(){
    return R;
}

float UnderwaterVehicle::GetWinchRPM(){
    return winchRPM_;
}

void UnderwaterVehicle::SetCableLength(const double &l){
    if(l <= Cable_params.length_max && l >= Cable_params.length_min)
        cable_length_released_ =l;

    for(int i =0; i < N_layer; i++)
        if(cable_length_released_ > CableLengthThreshold[i]){
            current_layer = i;
            break;
        }
    R = WindingRadiusPerLayer[current_layer];

}

void UnderwaterVehicle::InitializeMatrices(const Eigen::Vector6d &v_rel, const Eigen::RotationMatrix& worldF_R_bodyF){
/*
    if(!params.heavyConf){
        params.K_diag.conservativeResize(6,1);
        params.Q_diag.conservativeResize(6,1);
        K.resize(6,6);
        K = params.K_diag.asDiagonal();
        Q.resize(6,6);
        Q = params.Q_diag.asDiagonal();
        T.resize(6,6);
        T.row(0) = params.T_vector.segment(0,6);
        T.row(1) = params.T_vector.segment(6,6);
        T.row(2) = params.T_vector.segment(12,6);
        T.row(3) = params.T_vector.segment(18,6);
        T.row(4) = params.T_vector.segment(24,6);
        T.row(5) = params.T_vector.segment(30,6);
    }
    else{
        params.K_diag.conservativeResize(8,1);
        params.Q_diag.conservativeResize(8,1);
        K.resize(8,8);
        K = params.K_diag.asDiagonal();
        Q.resize(8,8);
        Q = params.Q_diag.asDiagonal();
        T.resize(6,8);
        T.row(0) = params.T_vector.segment(0,8);
        T.row(1) = params.T_vector.segment(8,8);
        T.row(2) = params.T_vector.segment(16,8);
        T.row(3) = params.T_vector.segment(24,8);
        T.row(4) = params.T_vector.segment(32,8);
        T.row(5) = params.T_vector.segment(40,8);
    }
*/
    M = getM();
    Minv = getInvM();
    C = getC(v_rel);
    D = getD(v_rel);
    bodyF_g = ComputeG_bodyF(worldF_R_bodyF);

    Cable_params.AttachPoint = {-params.L / 2, 0.0, 0.0};
}

void UnderwaterVehicle::InitializeCableWinch(){

    CircleNumberPerLayer = Cable_params.SpoolWidth / Cable_params.diameter;
    h = sqrt(3)/2 * Cable_params.diameter;
    h_total = 0;
    R = Cable_params.SpoolDiameter/2;

    float length, lengthPerLayer;
    length = Cable_params.length_max;
    lengthPerLayer = 2 * M_PI * R / 1000 * CircleNumberPerLayer;
    int i=0;
    MaxCableLengthPerLayer.resize(100);
    WindingRadiusPerLayer.resize(100);
    while(length > lengthPerLayer){
        WindingRadiusPerLayer[i] = R;
        MaxCableLengthPerLayer[i] = lengthPerLayer;
        length = length - lengthPerLayer;
        h_total = h_total + h;
        R = Cable_params.SpoolDiameter/2 + h_total;
        lengthPerLayer = 2 * M_PI * R / 1000 * CircleNumberPerLayer;
        i++;
    }
    MaxCableLengthPerLayer[i] = length;
    WindingRadiusPerLayer[i] = R;
    current_layer = i;
    N_layer = i+1;
    MaxCableLengthPerLayer.conservativeResize(N_layer);
    WindingRadiusPerLayer.conservativeResize(N_layer);

    CableLengthThreshold.resize(N_layer);
    CableLengthThreshold.setZero();
    Eigen::VectorXf v;
    v.resize(N_layer);
    for (int j = 1; j < N_layer; j++){
        v.setZero();
        v.segment(0, N_layer - j) = MaxCableLengthPerLayer.tail(N_layer - j);
        //CableLengthThreshold = CableLengthThreshold + CableLengthThreshold.tail(N_layer - j);
        CableLengthThreshold = CableLengthThreshold + v;
    }
    cable_length_released_ = 0;
    winchRPM_ = 0;
    std::cout << "CircleNumberPerLayer = "<< CircleNumberPerLayer<< std::endl;
    std::cout << "N_layer = "<< N_layer<< std::endl;
    std::cout << "h = "<< h<< std::endl;
    std::cout << "MaxCableLengthPerLayer = "<< MaxCableLengthPerLayer<< std::endl;
    std::cout << "CableLengthThreshold = "<< CableLengthThreshold<< std::endl;
    std::cout << "WindingRadiusPerLayer = "<< WindingRadiusPerLayer<< std::endl;
}

void UnderwaterVehicle::RunCableWinch(const float &rpm, float &velocity){
    float w = M_PI/30 * rpm;
    winchRPM_ = rpm;
    velocity = w * R / 1000;
}

void UnderwaterVehicle::UpdateCableLength(const float &percentage, const float &dt){
    float w = M_PI/30 * winchRPM_ * percentage;
    float v = w * R;
    cable_length_released_ = cable_length_released_ + v * dt;

    if(cable_length_released_ > Cable_params.length_max)
        cable_length_released_ = Cable_params.length_max;
    else if(cable_length_released_ < Cable_params.length_min)
        cable_length_released_ = Cable_params.length_min;
    else
        cable_length_released_ = 0;

    for(int i =0; i < N_layer; i++)
        if(cable_length_released_ > CableLengthThreshold[i]){
            current_layer = i;
            break;
        }
    R = WindingRadiusPerLayer[current_layer];
}

void UnderwaterVehicle::RunCableWinchToReachLength(const float &rpm, float &l, const float &dt){
    float precision = 1; float w;

    if(l > Cable_params.length_max)
        l = Cable_params.length_max;
    else if(l < Cable_params.length_min)
        l = Cable_params.length_min;

    if(abs(cable_length_released_ - l) > precision){
        if(cable_length_released_ < l){
            w = M_PI/30 * rpm;
            winchRPM_ = rpm;
        }
        else{
            w = - M_PI/30 * rpm;
            winchRPM_ = - rpm;
        }
        float v = w * R / 1000;
        UpdateCableLength(v,dt);
    }
    else winchRPM_ =0;

}

void UnderwaterVehicle::UpdateMatrices(const Eigen::Vector6d &v_rel, const Eigen::RotationMatrix& worldF_R_bodyF){
    C = getC(v_rel);
    D = getD(v_rel);
    bodyF_g = ComputeG_bodyF(worldF_R_bodyF);
}

Eigen::VectorXd UnderwaterVehicle::ThusterAllocation(const Eigen::Vector6d &tau){ // not needed anymore (functions moved to dynamic controller)

    //Eigen::MatrixXd m = T * K * Q;
    Eigen::MatrixXd m = params.T * params.K * params.Q;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd( m, Eigen::ComputeFullV | Eigen::ComputeFullU );
    Eigen::VectorXd volt = svd.solve(tau);

    return volt;
}

void UnderwaterVehicle::ThrustersSaturation(Eigen::VectorXd &thruster_force, const double& Saturation) // maybe we don't need it
{
    for (int i = 0 ;i < thruster_force.size() ; i++){
        if(thruster_force[i] > Saturation)
            thruster_force[i] = Saturation;
        else if(thruster_force[i] < -Saturation)
            thruster_force[i] = -Saturation;
    }

}

// not needed anymore (functions moved to dynamic controller)
void UnderwaterVehicle::Halt(Eigen::Vector6d &volt){
    volt.setZero();
}

void UnderwaterVehicle::Hold(Eigen::Vector6d &volt){

    Eigen::Vector6d tau = bodyF_g;
    Eigen::Vector6d tau_ref; tau_ref.setZero();
    volt = ThusterAllocation(-tau + tau_ref);
}

void UnderwaterVehicle::moveUp(Eigen::Vector6d &volt){
    Eigen::Vector6d tau = bodyF_g;
    Eigen::Vector6d tau_ref; tau_ref.setZero();
    tau_ref[2] = -10.0;
    volt = ThusterAllocation(-tau + tau_ref);
}

void UnderwaterVehicle::moveDown(Eigen::Vector6d &volt){
    Eigen::Vector6d tau = bodyF_g;
    Eigen::Vector6d tau_ref; tau_ref.setZero();
    tau_ref[2] = 10.0;
    volt = ThusterAllocation(-tau + tau_ref);
}

void UnderwaterVehicle::moveForward(Eigen::Vector6d &volt){
    //Eigen::Vector6d tau = bodyF_F_coriolis_drag + bodyF_g;
    Eigen::Vector6d tau = bodyF_g;
    Eigen::Vector6d tau_ref; tau_ref.setZero();
    tau_ref[0] = 10.0;
    volt = ThusterAllocation(-tau + tau_ref);
}

void UnderwaterVehicle::moveBackward(Eigen::Vector6d &volt){
    Eigen::Vector6d tau = bodyF_g;
    Eigen::Vector6d tau_ref; tau_ref.setZero();
    tau_ref[0] = -10.0;
    volt = ThusterAllocation(-tau + tau_ref);
}

void UnderwaterVehicle::moveLeft(Eigen::Vector6d &volt){
    Eigen::Vector6d tau = bodyF_g;
    Eigen::Vector6d tau_ref; tau_ref.setZero();
    tau_ref[5] = -0.1;
    volt = ThusterAllocation(-tau + tau_ref);
}

void UnderwaterVehicle::moveRight(Eigen::Vector6d &volt){
    Eigen::Vector6d tau = bodyF_g;
    Eigen::Vector6d tau_ref; tau_ref.setZero();
    tau_ref[5] = 0.1;
    volt = ThusterAllocation(-tau + tau_ref);
}

void UnderwaterVehicle::MoveByForce(const Eigen::Vector6d &force, Eigen::Vector6d &volt){
    Eigen::Vector6d tau = bodyF_g;
    //Eigen::Vector6d tau_ref; tau_ref.setZero();
    //tau_ref[5] = 0.1;
    volt = ThusterAllocation(-tau + force);
}
// all the previous functions are not needed anymore

Eigen::Vector6d UnderwaterVehicle::getg_bodyF(){
    return bodyF_g;
}

Eigen::Vector6d UnderwaterVehicle::getFcable_bodyF(){
    return bodyF_F_cable;
}

Eigen::Vector6d UnderwaterVehicle::getFthruster_bodyF(){
    return body_F_thruster;
}

Eigen::Vector6d UnderwaterVehicle::getCoriolisAndDrag_bodyF(){
    return bodyF_F_coriolis_drag;
}


