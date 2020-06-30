/*
 * MPC controller
 * Author: Chang, Xu
*/


#ifndef MPC_HH
#define MPC_HH
#include <Eigen/Dense>

/*
 * This MPC controller is applicable to single hard body which can have multi-contact points.
 *
 *Naming:
     * nF: number of forces, i.e., that of limbs.
     * pnF: Array pointer to nF.
     * robotmass: robot mass.
     * mu: friction coefficients.
     * fz_high: max support force.
     * fz_low: min support force.
     * timestep: time step for estimating future state variables
     * force_weights, attitude_weights, pos_weights: weights of force, attitude and position  in MPC
     * gravity: gravity acceleration vector
     * InertialInBody: Ic, i.e., inertial matrix according to center of mass.
 *
 * Application:
 * 1. Build up a BalanceController class object
 * 2. Call member function Init.
 * 3. Call member function SetMode to choose which variables are controlled.
 * 4. Call member function SetActivity to notify the class which limb is used to be supporting.
 * 5. Call member function Solve or SolveAddPIDCompensation
 *
*/
class BalanceController
{
    int nF;
    int ksteps;

    double robotmass = 100;
    double mu = 0.6;
    double timestep = 0.001;

    double fz_high = - 0.1;
    double fz_low= -700;

    double force_weights = 1e-6;
    Eigen::Vector3d attitude_weights;
    Eigen::Vector3d position_weights;
    Eigen::Vector3d anglesp_weights;
    Eigen::Vector3d vel_weights;
    Eigen::Matrix3d kF;
    Eigen::Matrix3d dF;
    Eigen::Matrix3d ktau;
    Eigen::Matrix3d dtau;
    double alphaF_regularization = 1e-2;
    double alphaTau_regularization = 1e-2;

    Eigen::Vector3d gravity;
    Eigen::Vector3d* pnF;

    Eigen::Matrix<double,15,15> A;
    Eigen::Matrix<double,15,12> B;
    Eigen::Matrix<double,15,15> L;
    Eigen::Matrix<double,12,12> K;
    Eigen::Matrix<double,15,15> At;
    Eigen::Matrix<double,15,12> Bt;
    Eigen::Matrix<double,15,  1> Xref;

    Eigen::Matrix3d InertialInBody;
    Eigen::Matrix3d BodytoWorldDCM;

    Eigen::VectorXd solution_UniformForceMPC;
    Eigen::VectorXd solution_PDCompensator;
    Eigen::VectorXd solution;

    Eigen::VectorXi supporting;
public:
    enum MODE
    {
        AttitudePosition,
        AttitudeVelocity,
        PositionAnglespeed,
        VelocityAnglespeed,
        WALK,
        JUMP,
        BOUND
    };
public:
    BalanceController(int nF);
    ~BalanceController();
    void Init(const Eigen::Matrix<double, 15, 1> X_ref, const Eigen::Vector3d nr[] , const int k_steps, const Eigen::Vector3d& atti_weights= Eigen::Vector3d::Identity(), const Eigen::Vector3d& p_weights= Eigen::Vector3d::Identity(),
              const Eigen::Vector3d& anglespeed_weights = Eigen::Vector3d::Identity(), const Eigen::Vector3d& velocity_weights = Eigen::Vector3d::Identity(), const double f_weights = 1e-6, const double robot_mass = 100);
    //TODO, return value
    void Solve(Eigen::VectorXd X0, Eigen::Vector3d rn[]);
    void SolveAddPIDCompensation(Eigen::VectorXd X0, Eigen::Vector3d rn[]);
    Eigen::VectorXd GetSolution();
    Eigen::VectorXd GetSolutionUniformForceMPC();
    Eigen::VectorXd GetSolutionPDCompensator();
    void CalculateDCM(Eigen::Vector3d angle);
    inline void SetActivity(const Eigen::Vector4i& support) { supporting = support;}
    void SetXref(const Eigen::VectorXd& xref);
    Eigen::VectorXd GetXref();
    void SetMode(MODE mode);
    void ResetPIDCompensationParameters();

};

#endif
