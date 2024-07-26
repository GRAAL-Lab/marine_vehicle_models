#include "ctrl_toolbox/HelperFunctions.h"
#include "libconfig.h++"
#include "rml/RML.h"

struct UnderwaterModelParameters{
    bool heavyConf; // ROV configuration (normal/heavy)
    float m; // mass of vehicle
    float rho; // water density
    float L; // Geometrical parameter
    float H; // Geometrical parameter
    float w; // Geometrical parameter
    //Eigen::MatrixXf diagXYZKMN[6];
    Eigen::Vector6d diagXYZKMN;
    Eigen::Vector6d M_a_diag;
    Eigen::Vector6d D_diag;
    Eigen::VectorXd K_diag;
    Eigen::VectorXd Q_diag;
    Eigen::Vector3d CG; // Center of Gravity
    Eigen::Vector3d CB; // Center of Boyancy
    Eigen::Vector3d Ixyz; // Ix, Iy and Iz

    float G; // Gravity constant
    float B; // Buoyance, buoyant force

    Eigen::VectorXd T_vector; // thruster allocation matrix written as a vector (to read it from conf file)
    std::vector<float> Tvec;
    Eigen::MatrixXd T; // thrust configuration matrix
    Eigen::MatrixXd K; // thrust coefficient
    Eigen::MatrixXd Q; // thrust coefficient weight


    UnderwaterModelParameters()
        : m(0.0)
        , rho(0.0)
        , L(0.0)
        , H(0.0)
        , w(0.0)
        , G(0.0)
        , B(0.0)

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

    friend std::ostream& operator<<(std::ostream& os, UnderwaterModelParameters const& a)
    {
        Eigen::IOFormat TabbedCleanFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "\t", "\n", "", "");
        return os << "Underwater Model Params:\n"
                  << "heavyMode: " << a.heavyConf << "\n"
                  << "m: " << a.m << "\n"
                  << "rho: " << a.rho << "\n"
                  << "L: " << a.L << "\n"
                  << "H: " << a.H << "\n"
                  << "G: " << a.G << "\n"
                  << "B: " << a.B << "\n"
                  << "W: " << a.w << "\n"
                  << "M_a_diag: " << a.M_a_diag << "\n"
                  << "D_diag: " << a.D_diag << "\n"
                  << "K_diag: " << a.K_diag << "\n"
                  << "Q_diag: " << a.Q_diag << "\n"
                  << "Center of Buoyancy: " << a.CB << "\n"
                  << "Center of Gravity: " << a.CG << "\n"
                  << "Ixyz: " << a.Ixyz << "\n";
    }

    bool LoadConfiguration(const libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& blueROVmodel = root["blueROVmodel"];

        if (!ctb::GetParam(blueROVmodel, heavyConf, "heavyConf"))
            return false;
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
        //if (!ctb::GetParamVector(blueROVmodel, K_diag, "K_diag"))
        //    return false;
        //if (!ctb::GetParamVector(blueROVmodel, Q_diag, "Q_diag"))
        //    return false;
        if (!ctb::GetParamVector(blueROVmodel, CG, "CG"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, CB, "CB"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, Ixyz, "Ixyz"))
            return false;

        //T_vector.resize(36);
        if(!heavyConf){
            T_vector.resize(36,1);
            if (!ctb::GetParamVector(blueROVmodel, T_vector, "T")) // can we read a matrix from con file??????? convert to matrix
                return false;
            T.resize(6,6);
            T.row(0) = T_vector.segment(0,6);
            T.row(1) = T_vector.segment(6,6);
            T.row(2) = T_vector.segment(12,6);
            T.row(3) = T_vector.segment(18,6);
            T.row(4) = T_vector.segment(24,6);
            T.row(5) = T_vector.segment(30,6);
            K_diag.resize(6,1);
            if (!ctb::GetParamVector(blueROVmodel, K_diag, "K_diag"))
                return false;
            K.resize(6,6);
            K = K_diag.asDiagonal();
            Q_diag.resize(6,1);
            if (!ctb::GetParamVector(blueROVmodel, Q_diag, "Q_diag"))
                return false;
            Q.resize(6,6);
            Q = Q_diag.asDiagonal();

        }
        else{
            T_vector.resize(48,1);
            if (!ctb::GetParamVector(blueROVmodel, T_vector, "T_heavy")) // can we read a matrix from con file??????? convert to matrix
                return false;
            T.resize(6,8);
            T.row(0) = T_vector.segment(0,8);
            T.row(1) = T_vector.segment(8,8);
            T.row(2) = T_vector.segment(16,8);
            T.row(3) = T_vector.segment(24,8);
            T.row(4) = T_vector.segment(32,8);
            T.row(5) = T_vector.segment(40,8);
            K_diag.resize(8,1);
            if (!ctb::GetParamVector(blueROVmodel, K_diag, "K_diag_heavy"))
                return false;
            K.resize(8,8);
            K = K_diag.asDiagonal();
            Q_diag.resize(8,1);
            if (!ctb::GetParamVector(blueROVmodel, Q_diag, "Q_diag_heavy"))
                return false;
            Q.resize(8,8);
            Q = Q_diag.asDiagonal();

        }
        //if (!ctb::GetParamVector(blueROVmodel, T_vector, "T")) // can we read a matrix from con file??????? convert to matrix
        //    return false;

        return true;
    }
};

struct CableParameters{

    float length_max;
    float length_min;
    float diameter;
    float stiffness;
    float SpoolWidth;
    float SpoolDiameter;
    float winch_rpm;

    /*float length;
    Eigen::Vector3d pos_starting;
    Eigen::Vector3d pos_ending;
    Eigen::Vector6d force;*/
    Eigen::Vector3d AttachPoint; // the cable fixing point on ROV

    CableParameters()
        : length_max(0.0)
        , length_min(0.0)
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

        if (!ctb::GetParam(blueROVmodel, length_max, "cable_length_max"))
            return false;
        if (!ctb::GetParam(blueROVmodel, length_min, "cable_length_min"))
            return false;
        if (!ctb::GetParam(blueROVmodel, diameter, "cable_diameter"))
            return false;
        if (!ctb::GetParam(blueROVmodel, stiffness, "cable_stiffness"))
            return false;
        if (!ctb::GetParam(blueROVmodel, SpoolWidth, "spool_width"))
            return false;
        if (!ctb::GetParam(blueROVmodel, SpoolDiameter, "spool_diameter"))
            return false;
        if (!ctb::GetParamVector(blueROVmodel, AttachPoint, "cable_AttachPoint"))
            return false;
        if (!ctb::GetParam(blueROVmodel, winch_rpm, "winch_rpm"))
            return false;

        return true;
    }
};
