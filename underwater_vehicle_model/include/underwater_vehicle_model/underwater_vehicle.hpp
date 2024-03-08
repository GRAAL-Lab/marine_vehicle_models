#ifndef UNDERWATER_VEHICLE_H
#define UNDERWATER_VEHICLE_H

#include "libconfig.h++"
#include "rml/RML.h"
#include "ctrl_toolbox/HelperFunctions.h"
#include "underwater_vehicle_model/model_data_structs.hpp"


class UnderwaterVehicle {

    /**
     * @brief Class of underwater vehicle model
     */
    void InverseMotorEquation(const Eigen::Vector6d& linAngVel, double& thrust_force, double& thruster_perc); // double& thruster or without&?????
    Eigen::Matrix3d get_TensorInertia();
    //bool LoadConfiguration(const libconfig::Config& confObj);
    Eigen::Matrix6d M_a; // added mass Matrix
    Eigen::Matrix6d M; // entire mass matrix
    Eigen::Matrix6d Minv; // inverse mass matrix
    //Eigen::MatrixXd K; // thrust coefficient
    //Eigen::MatrixXd Q; // thrust coefficient weight
    //Eigen::MatrixXd T; // thrust configuration matrix
    Eigen::Matrix6d C; // entire coriolis matrix
    Eigen::Matrix6d D; // entire damping matrix
    Eigen::Vector6d bodyF_F_coriolis_drag; // restoring force
    Eigen::Vector6d bodyF_g; // restoring force
    Eigen::Vector6d body_F_thruster; // thruster forces
    Eigen::Vector6d bodyF_F_cable; // cable forces
    Eigen::Vector6d u_motor; // motor commands
    Eigen::Matrix3d I0; // motor commands

    float cable_length_released; // ?
    int CircleNumberPerLayer;
    float h; // the vertical distance between two layer
    float h_total;
    float R; // Spool Radius
    int current_layer;
    int N_layer; // total number of cable layer on the spool
    float winchRPM;
    Eigen::VectorXf MaxCableLengthPerLayer;
    Eigen::VectorXf WindingRadiusPerLayer;
    Eigen::VectorXf CableLengthThreshold;
    //Eigen::Vector3d startingPos_cable; // ?
    //Eigen::Vector3d endingPos_cable; // ?

public:
    UnderwaterModelParameters params;
    CableParameters Cable_params;

    UnderwaterVehicle();
    Eigen::Vector3d ComputeCoriolisAndDragForces(Eigen::Vector6d vel);
    double GetThrusterForce(double n, double linXVel);
    double PercentageToRPM(double h);
    double RPMToPercentage(double n);
    //void DirectDynamics(const Eigen::Vector6d& volt_bodyF, const Eigen::Vector6d& bodyF_F_cable, const Eigen::Vector6d& eta, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    void DirectDynamics(const Eigen::VectorXd& volt, const Eigen::Vector6d& bodyF_F_cable,
                        const Eigen::RotationMatrix& worldF_R_bodyF, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    //Eigen::Vector2d ThusterAllocation(Eigen::Vector2d& tau);
    void InverseMotorsEquations(const Eigen::Vector6d& linAngVel, Eigen::Vector2d thrust_force, double& h_p, double& h_s);
    //void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut);
    //void ThrusterDynamicAllocator(const double f_des, const double n_des, double& h_s, double &h_p);
    Eigen::Matrix6d getM();
    Eigen::Matrix6d getInvM();
    Eigen::Matrix6d getC(const Eigen::Vector6d &v_ref);
    Eigen::Matrix6d getD(const Eigen::Vector6d &v_rel);
    Eigen::Vector6d ComputeG_bodyF(const Eigen::RotationMatrix& worldF_R_bodyF);
    void ThrustersSaturation(Eigen::VectorXd &thruster_force, const double& Saturation); // maybe we don't need it
    Eigen::VectorXd ThusterAllocation(const Eigen::Vector6d& tau);
    void Halt(Eigen::Vector6d &volt);
    void Hold(Eigen::Vector6d &volt);
    void moveUp(Eigen::Vector6d &volt);
    void moveDown(Eigen::Vector6d &volt);
    void moveLeft(Eigen::Vector6d &volt);
    void moveRight(Eigen::Vector6d &volt);
    void moveForward(Eigen::Vector6d &volt);
    void moveBackward(Eigen::Vector6d &volt);
    void MoveByForce(const Eigen::Vector6d &force, Eigen::Vector6d &volt);

    Eigen::Vector6d getCoriolisAndDrag_bodyF();
    Eigen::Vector6d getg_bodyF();
    Eigen::Vector6d getFcable_bodyF();
    Eigen::Vector6d getFthruster_bodyF();

    //void UpdateMatrices(const Eigen::Vector6d &v_rel, const Eigen::Vector6d &eta);
    void UpdateMatrices(const Eigen::Vector6d &v_rel, const Eigen::RotationMatrix& worldF_R_bodyF);
    void InitializeMatrices(const Eigen::Vector6d &v_rel, const Eigen::RotationMatrix& worldF_R_bodyF);
    void InitializeCableWinch();
    void RunCableWinch(const float &rpm, float &velocity);
    void UpdateCableLength(const float &v, const float &dt);
    Eigen::Vector6d VoltageToForces(const Eigen::Vector6d& volt);

    Eigen::Vector6d ComputeFcable_bodyF(const Eigen::Vector3d &s_pos_worldF, const Eigen::Vector3d &e_pos_worldF, const float &length, const Eigen::RotationMatrix &worldF_R_bodyF);
    //double GetCablePos_starting();
    //double GetCablePos_ending();
    float GetCableReleasedLength();
    float GetCableLayer();
    float GetCableWindingRadius();
    float GetWinchRPM();

    //void SetCablePos_starting(const Eigen::Vector3d &pos);
    //void SetCablePos_ending(const Eigen::Vector3d &pos);
    void SetCableLength(const double &l);
    void RunCableWinchToReachLength(const float &rpm, float &l, const float &dt);
};

#endif // UNDERWATER_VEHICLE_H
