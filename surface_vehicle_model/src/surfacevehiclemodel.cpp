#include "surface_vehicle_model/surfacevehiclemodel.hpp"

/**
 * Helper clipping function
 */

bool SolveSecondOrderEquation(double a, double b, double c, std::pair<double, double>& solutions)
{
    double delta = b * b - 4.0 * a * c;
    if (delta < 0.0) {
        std::cerr << "Error solving 2nd order eq: delta < 0.0!!! Returning 0.0" << std::endl;
        solutions = std::make_pair<double, double>(0.0, 0.0);
        return false;
    } else {
        solutions.first = (-b + std::sqrt(delta)) / (2.0 * a);
        solutions.second = (-b - std::sqrt(delta)) / (2.0 * a);
        return true;
    }
}

SurfaceVehicleModel::SurfaceVehicleModel() { }

double SurfaceVehicleModel::GetThrusterForce(double n, double linXVel)
{
    double force;
    if (n > 0.0) {
        force = params.b1_pos * n * n - params.b2_pos * n * linXVel;
    } else {
        force = -params.b1_neg * n * n + params.b2_neg * n * linXVel;
    }
    return force;
}

double SurfaceVehicleModel::PercentageToRPM(double h)
{
    double rpm;
    if (h > 0.0) {
        rpm = h * params.lambda_pos;
    } else {
        rpm = h * params.lambda_neg;
    }
    return rpm;
}

double SurfaceVehicleModel::RPMToPercentage(double rpm)
{
    double h;
    if (rpm > 0.0) {
        h = rpm / params.lambda_pos;
    } else {
        h = rpm / params.lambda_neg;
    }
    return h;
}

void SurfaceVehicleModel::DirectDynamics(double h_p, double h_s, double& n_p, double& n_s, const Eigen::Vector6d& linAngVel, Eigen::Vector6d& linAngAcc)
{
    Eigen::Vector3d nir, tauStar;
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
    linAngAcc(5) = nir(2);

    /*
    std::cout << "tauX_: " << tauX_ << std::endl;
    std::cout << "tauN_: " << tauN_ << std::endl;std::cout << "tauC: " << tauC.transpose() << std::endl;
    std::cout << "tauStar: " << tauStar_.transpose() << std::endl;
    std::cout << "I_:\n" << params.Inertia << std::endl;
    std::cout << "I_pinv:\n" << I_pinv << std::endl;
    std::cout << "nir: " << nir_.transpose() << std::endl;
    std::cout << "linAngAcc: " << linAngAcc.transpose() << std::endl;
    */
}

Eigen::Vector3d SurfaceVehicleModel::ComputeCoriolisAndDragForces(Eigen::Vector6d vehvel)
{
    double alpha;

    if (vehvel[0] + vehvel[5] * (params.d + params.hullWidth / 2) < 0 && vehvel[0] + vehvel[5] * (params.d - params.hullWidth / 2) < 0)
        alpha = 0;
    else if (vehvel[0] - vehvel[5] * (params.d + params.hullWidth / 2) < 0 && vehvel[0] - vehvel[5] * (params.d - params.hullWidth / 2) < 0)
        alpha = 0;
    else if (vehvel[0] + vehvel[5] * (params.d + params.hullWidth / 2) < 0 || vehvel[0] + vehvel[5] * (params.d - params.hullWidth / 2) < 0)
        alpha = (-vehvel[0] / vehvel[5] - (params.d - params.hullWidth / 2)) / 0.4;
    else if (vehvel[0] - vehvel[5] * (params.d + params.hullWidth / 2) < 0 || vehvel[0] - vehvel[5] * (params.d - params.hullWidth / 2) < 0)
        alpha = (vehvel[0] / vehvel[5] - (params.d - params.hullWidth / 2)) / params.hullWidth;
    else
        alpha = 1;

    double neg_tauN = params.cNneg[0] * vehvel[0] * vehvel[5] + params.cNneg[1] * vehvel[5] + params.cNneg[2] * vehvel[5] * abs(vehvel[5]);
    double pos_tauN = params.cN[0] * vehvel[0] * vehvel[5] + params.cN[1] * vehvel[5] + params.cN[2] * vehvel[5] * abs(vehvel[5]);

    return { params.cX[0] * vehvel[5] * vehvel[5] + params.cX[1] * vehvel[0] + params.cX[2] * vehvel[0] * abs(vehvel[0]), params.cY[0] * vehvel[5] * vehvel[5] + params.cY[1] * vehvel[1] + params.cY[2] * vehvel[1] * abs(vehvel[1]), alpha * pos_tauN + (1 - alpha) * neg_tauN };
}

// Paper [Simetti,Indiveri (2022)] Eq.28
Eigen::Vector2d SurfaceVehicleModel::ThusterAllocation(Eigen::Vector2d& tau)
{
    double forceP;
    double forceS;
    double k_p;
    double k_s;

    k_p = params.k_pos;
    k_s = params.k_pos;
    forceP = ((params.d - k_s * params.l) * tau[0] + tau[1]) / (2 * params.d + k_p * params.l - k_s * params.l);
    forceS = ((params.d + k_p * params.l) * tau[0] - tau[1]) / (2 * params.d + k_p * params.l - k_s * params.l);
    if (forceP >= 0 && forceS >= 0)
        return { forceP, forceS };

    k_p = params.k_pos;
    k_s = params.k_neg;
    forceP = ((params.d - k_s * params.l) * tau[0] + tau[1]) / (2 * params.d + k_p * params.l - k_s * params.l);
    forceS = ((params.d + k_p * params.l) * tau[0] - tau[1]) / (2 * params.d + k_p * params.l - k_s * params.l);
    if (forceP >= 0 && forceS <= 0)
        return { forceP, forceS };

    k_p = params.k_neg;
    k_s = params.k_neg;
    forceP = ((params.d - k_s * params.l) * tau[0] + tau[1]) / (2 * params.d + k_p * params.l - k_s * params.l);
    forceS = ((params.d + k_p * params.l) * tau[0] - tau[1]) / (2 * params.d + k_p * params.l - k_s * params.l);
    if (forceP <= 0 && forceS <= 0)
        return { forceP, forceS };

    k_p = params.k_neg;
    k_s = params.k_pos;
    forceP = ((params.d - k_s * params.l) * tau[0] + tau[1]) / (2 * params.d + k_p * params.l - k_s * params.l);
    forceS = ((params.d + k_p * params.l) * tau[0] - tau[1]) / (2 * params.d + k_p * params.l - k_s * params.l);
    if (forceP <= 0 && forceS >= 0)
        return { forceP, forceS };

    return {0, 0};
    //forceP = 0.5 * (tau[0] + tau[1] / params.d);
    //forceS = 0.5 * (tau[0] - tau[1] / params.d);
}

void SurfaceVehicleModel::InverseMotorsEquations(const Eigen::Vector6d& linAngVel, Eigen::Vector2d thrust_force, double& h_p, double& h_s)
{
    Eigen::Vector6d inputVel = linAngVel;

    InverseMotorEquation(inputVel, thrust_force[0], h_p);
    //    std::cout << "-LEFT MOTOR MAPPING-" << std::endl;
    //    std::cout << "Req Vel: " << inputVel.transpose() << std::endl;
    //    std::cout << "tauX_: " << GetTauX() << std::endl;
    //    std::cout << "tauN_: " << GetTauN() << std::endl;
    //    std::cout << "h_p: " << h_p << std::endl;

    Eigen::Vector6d inputVel_s = inputVel;
    inputVel_s(5) = -inputVel_s(5);
    InverseMotorEquation(inputVel_s, thrust_force[1], h_s);
    //    std::cout << "-RIGHT MOTOR MAPPING-" << std::endl;
    //    std::cout << "Req Vel: " << inputVel_s.transpose() << std::endl;
    //    std::cout << "tauX_: " << GetTauX() << std::endl;
    //    std::cout << "tauN_: " << GetTauN() << std::endl;
    //    std::cout << "h_s: " << h_s << std::endl;

    //    std::cout << "****************************" << std::endl;
}

void SurfaceVehicleModel::InverseMotorEquation(const Eigen::Vector6d& linAngVel, double thrust_force, double& thruster_perc)
{
    double motorlinearXVel = linAngVel(0) + linAngVel(5) * params.d;
    double rpmsolution;

    std::pair<double, double> solutions;
    if (thrust_force > 0.0) {
        SolveSecondOrderEquation(params.b1_pos, -params.b2_pos * motorlinearXVel, -thrust_force, solutions);
        if (solutions.first > 0.0) {
            rpmsolution = solutions.first;
        } else {
            rpmsolution = solutions.second;
        }
    } else {
        SolveSecondOrderEquation(-params.b1_neg, params.b2_neg * motorlinearXVel, -thrust_force, solutions);
        if (solutions.first < 0.0) {
            rpmsolution = solutions.first;
        } else {
            rpmsolution = solutions.second;
        }
    }
    thruster_perc = RPMToPercentage(rpmsolution);
}
void SurfaceVehicleModel::ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut)
{
    //std::cout << "min-max: " << thMin << ", " << thMax << std::endl;
    double factor = 1.0;
    if (lThruster > thMax) {
        double newFactor = thMax / lThruster;
        if (newFactor < factor) {
            factor = newFactor;
        }
    } else if (lThruster < thMin) {
        double newFactor = thMin / lThruster;
        if (newFactor < factor) {
            factor = newFactor;
        }
    }

    if (rThruster > thMax) {
        double newFactor = thMax / rThruster;
        if (newFactor < factor) {
            factor = newFactor;
        }
    } else if (rThruster < thMin) {
        double newFactor = thMin / rThruster;
        if (newFactor < factor) {
            factor = newFactor;
        }
    }

    lSatOut = lThruster * factor;
    rSatOut = rThruster * factor;
}
/*void SurfaceVehicleModel::ThrusterDynamicAllocator(const double f_des, const double n_des, double &h_s, double& h_p){
}*/
