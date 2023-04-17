#ifndef UNDERWATER_VEHICLE_MODEL_H
#define UNDERWATER_VEHICLE_MODEL_H

#include "ctrl_toolbox/HelperFunctions.h"
#include "eigen3/Eigen/Dense"
#include "libconfig.h++"
#include "rml/RML.h"

struct UnderwaterModelParameters{
    float m; // mass of vehicle
    float rho; // water density
    float L; // Geometrical parameter
    float H; // Geometrical parameter
    //Eigen::MatrixXf diagXYZKMN[6];
    Eigen::VectorXf diagXYZKMN[6];
    Eigen::VectorXf M_a_diag[6];
    Eigen::VectorXf D_diag[6];
    Eigen::VectorXf K_diag[6];
    Eigen::Vector3f CG; // Center of Gravity
    Eigen::Vector3f CB; // Center of Boyancy

    float G; // Gravity constant
    float B; // Buoyance, buoyant force

    Eigen::MatrixXf T; // thruster allocation matrix
    Eigen::VectorXf u_motor; // motor commands


    UnderwaterModelParameters()
        : m(0.0)
        , rho(0.0)
        , L(0.0)
        , H(0.0)
        , G(0.0)
        , B(0.0)
    {
        //diagXYZKMN.setZero();
        //M_a_diag.setZero();
        //D_diag.setZero();

        CG.setZero();
        CB.setZero();
        //M_a.setZero();
        //M.setZero();
        //C.setZero();
        //D.setZero();

        //K.setZero();
        u_motor.setZero();
        //g.setZero();
    }

    bool LoadConfiguration(const libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& blueROVmodel = root["blueROVmodel"];

        if (!ctb::GetParamVector(blueROVmodel, m, "m"))
            return false;



        return true;
    }
};

class Underwater_Vehicle_Model {

    /**
     * @brief Class of underwater vehicle model
     */
    void InverseMotorEquation(const Eigen::Vector6d& linAngVel, double& thrust_force, double& thruster_perc); // double& thruster or without&?????
    Eigen::Matrix3f get_TensorInertia();
    //bool LoadConfiguration(const libconfig::Config& confObj);
    Eigen::MatrixXf M_a; // added mass Matrix
    Eigen::MatrixXf M; // entire mass matrix
    Eigen::MatrixXf K; // thrust coefficient
    Eigen::MatrixXf C; // entire coriolis matrix
    Eigen::MatrixXf D; // entire damping matrix
    Eigen::VectorXf g; // restoring force
    Eigen::VectorXf F; // thruster forces

public:
    UnderwaterModelParameters params;

    Underwater_Vehicle_Model();
    Eigen::Vector3d ComputeCoriolisAndDragForces(Eigen::Vector6d vel);
    double GetThrusterForce(double n, double linXVel);
    double PercentageToRPM(double h);
    double RPMToPercentage(double n);
    void DirectDynamics(const Eigen::Vector6d& volt,const Eigen::Vector6d& linAngVel_, const Eigen::Vector6d& eta, Eigen::Vector6d& linAngAcc_);
    Eigen::Vector2d ThusterAllocation(Eigen::Vector2d& tau);
    void InverseMotorsEquations(const Eigen::Vector6d& linAngVel, Eigen::Vector2d thrust_force, double& h_p, double& h_s);
    void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut);
    //void ThrusterDynamicAllocator(const double f_des, const double n_des, double& h_s, double &h_p);
    Eigen::MatrixXf getM();
    Eigen::MatrixXf getC(const Eigen::VectorXf &v_ref);
    Eigen::MatrixXf getD(const Eigen::VectorXf &v_rel);
    Eigen::MatrixXf getg(const Eigen::VectorXf &eta);
    void UpdateMatrices(const Eigen::VectorXf &v_rel,const Eigen::VectorXf &eta);
    void InitializeMatrices(const Eigen::VectorXf &v_rel,const Eigen::VectorXf &eta);
    Eigen::VectorXf VoltageToForces(const Eigen::VectorXf& volt);
};

#endif // UNDERWATER_VEHICLE_MODEL_H
