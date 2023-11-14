#ifndef UNDERWATER_VEHICLE_H
#define UNDERWATER_VEHICLE_H

#include "ctrl_toolbox/HelperFunctions.h"
#include "eigen3/Eigen/Dense"
#include "libconfig.h++"
#include "rml/RML.h"

struct UnderwaterModelParameters{
    float m; // mass of vehicle
    float rho; // water density
    float L; // Geometrical parameter
    float H; // Geometrical parameter
    float w; // Geometrical parameter
    //Eigen::MatrixXf diagXYZKMN[6];
    Eigen::Vector6d diagXYZKMN;
    Eigen::Vector6d M_a_diag;
    Eigen::Vector6d D_diag;
    Eigen::Vector6d K_diag;
    Eigen::Vector3d CG; // Center of Gravity
    Eigen::Vector3d CB; // Center of Boyancy
    Eigen::Vector3d Ixyz; // Ix, Iy and Iz

    float G; // Gravity constant
    float B; // Buoyance, buoyant force

    Eigen::VectorXd T_vector; // thruster allocation matrix written as a vector (to read it from conf file)
    std::vector<float> Tvec;

    UnderwaterModelParameters()
        : m(0.0)
        , rho(0.0)
        , L(0.0)
        , H(0.0)
        , G(0.0)
        , B(0.0)
        , w(0.0)
    {
        //diagXYZKMN.setZero();
        //M_a_diag.setZero();
        //D_diag.setZero();

        //CG.setZero();
        //CB.setZero();
        //M_a.setZero();
        //M.setZero();
        //C.setZero();
        //D.setZero();

        //K.setZero();
        //u_motor.setZero();
        //g.setZero();
    }

    bool LoadConfiguration(const libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& blueROVmodel = root["blueROVmodel"];

        if (!ctb::GetParam(blueROVmodel, m, "m"))
            return false;
        if (!ctb::GetParam(blueROVmodel, rho, "rho"))
            return false;
        if (!ctb::GetParam(blueROVmodel, L, "L"))
            return false;
        if (!ctb::GetParam(blueROVmodel, H, "H"))
            return false;
        if (!ctb::GetParam(blueROVmodel, w, "W"))
            return false;
        if (!ctb::GetParam(blueROVmodel, G, "G"))
            return false;
        if (!ctb::GetParam(blueROVmodel, B, "B"))
            return false;

        if (!ctb::GetParamVector(blueROVmodel, diagXYZKMN, "diagXYZKMN"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, M_a_diag, "M_a_diag"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, D_diag, "D_diag"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, K_diag, "K_diag"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, CG, "CG"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, CB, "CB"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, Ixyz, "Ixyz"))
            return false;

        //T_vector.resize(36);
        if (!ctb::GetParamVector(blueROVmodel, T_vector, "T")) // can we read a matrix from con file??????? convert to matrix
            return false;

        return true;
    }
};

struct CableParameters{

    float length_full;
    float diameter;
    float stiffness;

    /*float length;
    Eigen::Vector3d pos_starting;
    Eigen::Vector3d pos_ending;
    Eigen::Vector6d force;*/
    Eigen::Vector3d AttachPoint; // the cable fixing point on ROV

    CableParameters()
        : length_full(0.0)
        , diameter(0.0)
        , stiffness(0.0)
    //    , length(0.0)
    {
    //    pos_starting.setZero();
    //    pos_ending.setZero();
    //    force.setZero();
        AttachPoint.setZero();
    }

    bool LoadConfiguration(const libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& blueROVmodel = root["blueROVmodel"];

        if (!ctb::GetParam(blueROVmodel, length_full, "cable_length_full"))
            return false;
        if (!ctb::GetParam(blueROVmodel, diameter, "cable_diameter"))
            return false;
        if (!ctb::GetParam(blueROVmodel, stiffness, "cable_stiffness"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, AttachPoint, "cable_AttachPoint"))
            return false;

        return true;
    }
};

class Underwater_Vehicle {

    /**
     * @brief Class of underwater vehicle model
     */
    void InverseMotorEquation(const Eigen::Vector6d& linAngVel, double& thrust_force, double& thruster_perc); // double& thruster or without&?????
    Eigen::Matrix3d get_TensorInertia();
    //bool LoadConfiguration(const libconfig::Config& confObj);
    Eigen::Matrix6d M_a; // added mass Matrix
    Eigen::Matrix6d M; // entire mass matrix
    Eigen::Matrix6d Minv; // inverse mass matrix
    Eigen::Matrix6d K; // thrust coefficient
    Eigen::Matrix6d T; // thrust configuration matrix
    Eigen::Matrix6d C; // entire coriolis matrix
    Eigen::Matrix6d D; // entire damping matrix
    Eigen::Vector6d g_bodyF; // restoring force
    Eigen::Vector6d F_thruster; // thruster forces
    Eigen::Vector6d F_cable; // cable forces
    Eigen::Vector6d u_motor; // motor commands
    Eigen::Matrix3d I0; // motor commands

    float cable_length; // ?
    //Eigen::Vector3d startingPos_cable; // ?
    //Eigen::Vector3d endingPos_cable; // ?

public:
    UnderwaterModelParameters params;
    CableParameters Cable_params;

    Underwater_Vehicle();
    Eigen::Vector3d ComputeCoriolisAndDragForces(Eigen::Vector6d vel);
    double GetThrusterForce(double n, double linXVel);
    double PercentageToRPM(double h);
    double RPMToPercentage(double n);
    //void DirectDynamics(const Eigen::Vector6d& volt_bodyF, const Eigen::Vector6d& Fcable_bodyF, const Eigen::Vector6d& eta, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    void DirectDynamics(const Eigen::Vector6d& volt_bodyF, const Eigen::Vector6d& Fcable_bodyF,
                        const Eigen::RotationMatrix& worldF_R_bodyF, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    Eigen::Vector2d ThusterAllocation(Eigen::Vector2d& tau);
    void InverseMotorsEquations(const Eigen::Vector6d& linAngVel, Eigen::Vector2d thrust_force, double& h_p, double& h_s);
    void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut);
    //void ThrusterDynamicAllocator(const double f_des, const double n_des, double& h_s, double &h_p);
    Eigen::Matrix6d getM();
    Eigen::Matrix6d getInvM();
    Eigen::Matrix6d getC(const Eigen::Vector6d &v_ref);
    Eigen::Matrix6d getD(const Eigen::Vector6d &v_rel);
    Eigen::Vector6d getg_bodyF(const Eigen::RotationMatrix& worldF_R_bodyF);
    //void UpdateMatrices(const Eigen::Vector6d &v_rel, const Eigen::Vector6d &eta);
    void UpdateMatrices(const Eigen::Vector6d &v_rel, const Eigen::RotationMatrix& worldF_R_bodyF);
    void InitializeMatrices(const Eigen::Vector6d &v_rel, const Eigen::RotationMatrix& worldF_R_bodyF);
    Eigen::Vector6d VoltageToForces(const Eigen::Vector6d& volt);

    Eigen::Vector6d ComputeFcable_bodyF(const Eigen::Vector3d &s_pos_worldF, const Eigen::Vector3d &e_pos_worldF, const float &length, const Eigen::RotationMatrix &worldF_R_bodyF, const Eigen::Vector6d& linAngVel_);
    //double GetCablePos_starting();
    //double GetCablePos_ending();
    float GetCableCurrentLength();

    //void SetCablePos_starting(const Eigen::Vector3d &pos);
    //void SetCablePos_ending(const Eigen::Vector3d &pos);
    void SetCableLength(const double &l);
};

#endif // UNDERWATER_VEHICLE_H
