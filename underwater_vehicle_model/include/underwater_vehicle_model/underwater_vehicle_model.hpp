#ifndef UNDERWATER_VEHICLE_MODEL_H
#define UNDERWATER_VEHICLE_MODEL_H

#include "ctrl_toolbox/HelperFunctions.h"
#include "eigen3/Eigen/Dense"
#include "libconfig.h++"
#include "rml/RML.h"

class Underwater_Vehicle_Model {

    /**
     * @brief Class of ulisse model
     */
    void InverseMotorEquation(const Eigen::Vector6d& linAngVel, double thrust_force, double& thruster_perc);
    Eigen::Matrix3f get_TensorInertia();
    bool LoadConfiguration(const libconfig::Config& confObj);
    float m; // mass of vehicle
    float rho; // water density
    float L; // Geometrical parameter
    float H; // Geometrical parameter
    //Eigen::MatrixXf diagXYZKMN[6];
    Eigen::Vector3f diagXYZKMN[6];
    Eigen::VectorXf M_a_diag[6];
    Eigen::VectorXf D_diag[6];

    Eigen::Vector3f CG; // Center of Gravity
    Eigen::Vector3f CB; // Center of Boyancy
    Eigen::MatrixXf M_a; // added mass Matrix
    float G; // Gravity constant
    float B; // Buoyance, restoring force

public:
    //UlisseModelParameters params;
    Underwater_Vehicle_Model();
    Eigen::Vector3d ComputeCoriolisAndDragForces(Eigen::Vector6d vel);
    double GetThrusterForce(double n, double linXVel);
    double PercentageToRPM(double h);
    double RPMToPercentage(double n);
    void DirectDynamics(double h_p, double h_s, double& n_p, double& n_s, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    Eigen::Vector2d ThusterAllocation(Eigen::Vector2d& tau);
    void InverseMotorsEquations(const Eigen::Vector6d& linAngVel, Eigen::Vector2d thrust_force, double& h_p, double& h_s);
    void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut);
    //void ThrusterDynamicAllocator(const double f_des, const double n_des, double& h_s, double &h_p);
    Eigen::MatrixXf getM();
    Eigen::MatrixXf getC(const Eigen::VectorXf &v_ref);
    Eigen::MatrixXf getD(const Eigen::VectorXf &v_rel);
    Eigen::MatrixXf getg(const Eigen::VectorXf &eta);
};

#endif // UNDERWATER_VEHICLE_MODEL_H
