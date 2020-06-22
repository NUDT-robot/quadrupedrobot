#include <iostream>
#include <vector>
#include <cmath>
#include <qpOASES.hpp>

#include "include/mpc.hh"
#include "include/ds/traprocgen.hh"

using namespace Eigen;
using namespace std;
MPCController::MPCController(int n_F) : solution(3*n_F), solution_UniformForceMPC(3*n_F), solution_PDCompensator(3*n_F), supporting(n_F)
{
    nF = n_F;
    gravity << 0, 0, 9.8;
    InertialInBody << 1.95,     0,         0   ,
                                          0,     9.48,      0   ,
                                          0,        0,    10.11;

    pnF = new Vector3d[nF];
    ResetPIDCompensationParameters();
}

MPCController::~MPCController()
{
    delete [] pnF;
}

void MPCController::Init(const Matrix<double, 15, 1> X_ref, const Vector3d nr[] , const int k_steps, const Eigen::Vector3d& atti_weights, const Eigen::Vector3d& p_weights,
                         const Eigen::Vector3d& anglespeed_weights, const Eigen::Vector3d& velocity_weights, const double f_weights, const double robot_mass)
{
    Xref = X_ref;
    for(int i  = 0; i < nF; i++)
    {
        pnF[i] = nr[i];
    }
    ksteps = k_steps;
    robotmass = robot_mass;
    force_weights = f_weights;
    attitude_weights = atti_weights;
    position_weights = p_weights;
    anglesp_weights = anglespeed_weights;
    vel_weights = velocity_weights;
    A = MatrixXd::Zero(15, 15);
    B = MatrixXd::Zero(15, 12);
    L = MatrixXd::Identity(15, 15);

    L.block<3,3>(0, 0) = atti_weights.asDiagonal();
    L.block<3,3>(3, 3) = p_weights.asDiagonal();
    L.block<3,3>(6, 6) = anglespeed_weights.asDiagonal();
    L.block<3,3>(9, 9) = velocity_weights.asDiagonal();
    K = MatrixXd::Identity(12, 12)*force_weights;
}

void MPCController::Solve(VectorXd X0, Vector3d rn[])
{
    double* m = new double[nF];
    for(int i = 0; i < nF; i++)
        m[i] = robotmass;
    //std::cout << "MPC Solve ..." << std::endl << "X0 = "  << std::endl << X0 << std::endl << "Xref = " << std::endl<< Xref << std::endl;
    CalculateDCM(X0.segment(0,3));
// 1. Initialize Matrix
     //std::cout << "MPC Initialize Matrix ..." << std::endl;

    Matrix3d InertiaWorldInverse, R;
    Vector3d angle(X0[0], X0[1], X0[2]), dangle;
    Vector3d* r = new Vector3d[nF];

    // Calculating R
    R << cos(angle.y())*cos(angle.z())  ,  -sin(angle.z()),  0,
              cos(angle.y())*sin(angle.z())  ,  cos(angle.z()),   0,
                                 -sin(angle.y())          ,               0 ,            1;

    //Initialize matrix A0 , B0
    At << Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3),                   R.inverse()            , Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3),
                  Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Identity(3,3),  Eigen::Matrix3d::Zero(3,3),
                  Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3),
                  Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Identity(3,3),
                  Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3);
    InertiaWorldInverse.noalias() = (BodytoWorldDCM*InertialInBody*BodytoWorldDCM.transpose()).inverse();
    for(int i = 0; i < nF; i++)
    {
        r[i] = rn[i];
    }
    Bt << Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3),
                  Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3),
                  InertiaWorldInverse*MyMathFunc::CrossProductMatrixFromVector(r[0]),
                  InertiaWorldInverse*MyMathFunc::CrossProductMatrixFromVector(r[1]),
                  InertiaWorldInverse*MyMathFunc::CrossProductMatrixFromVector(r[2]),
                  InertiaWorldInverse*MyMathFunc::CrossProductMatrixFromVector(r[3]),
                  Eigen::Matrix3d::Identity(3,3)/m[0], Eigen::Matrix3d::Identity(3,3)/m[1], Eigen::Matrix3d::Identity(3,3)/m[2], Eigen::Matrix3d::Identity(3,3)/m[3],
                  Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3), Eigen::Matrix3d::Zero(3,3);

    Eigen::Vector3d pos(0,0,0), in_angularspeed, in_velocity;
    in_angularspeed = Xref.segment(6,3);
    in_velocity = Xref.segment(9,3);
    std::vector<Eigen::MatrixXd> avec, bvec;
    Eigen::MatrixXd MI;
    MI = Eigen::MatrixXd::Identity(15,15);
    avec.push_back(At);
    avec[0] += MI;
    bvec.push_back(Bt);
    for(int t = 1; t < ksteps; t++)
    {
        dangle = R*in_angularspeed;
        angle += dangle*timestep;
        //Reference Trajectories State Generation
        R   << cos(angle.y())*cos(angle.z()), -sin(angle.z()), 0,
                    cos(angle.y())*sin(angle.z()) , cos(angle.z()),  0,
                                     -sin(angle.y())           ,          0              ,  1;
        pos += in_velocity*timestep;
        At.block<3,3>(0, 6) << R.inverse();
        avec[0] =  (At+ MI) * (avec[0]);
        CalculateDCM(angle);
        InertiaWorldInverse.noalias() = (BodytoWorldDCM*InertialInBody*BodytoWorldDCM.transpose()).inverse();
        Bt.block<3,12>(6, 0) << InertiaWorldInverse*MyMathFunc::CrossProductMatrixFromVector(r[0]),
                                                      InertiaWorldInverse*MyMathFunc::CrossProductMatrixFromVector(r[1]),
                                                      InertiaWorldInverse*MyMathFunc::CrossProductMatrixFromVector(r[2]),
                                                      InertiaWorldInverse*MyMathFunc::CrossProductMatrixFromVector(r[3]);
        bvec[0] =  (At+ MI) * (bvec[0])+Bt;
    }
    A << avec[0];
    B << bvec[0];
    //std::cout << "Solve matrix A: " << A << std::endl << "Matrix B:" << B << std::endl;
    delete [] r;
// 2. Slove MPC formulation using qpOASES
    //Note: Eigen's matrix is column-major storage order, and is differenet from qpOASES which is row-major. Make sure that using Eigen with row-major instead.
    USING_NAMESPACE_QPOASES
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Hm(3*nF, 3*nF);
    Eigen::RowVectorXd gv(3*nF);
    Hm.noalias() = 2*(B.transpose()*L*B+K);
    gv.noalias() = 2*B.transpose()*L*(A*X0-Xref);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ConstaintM(4*nF, 3*nF);
    Eigen::VectorXd lbA(4*nF), ubA(4*nF), lb(3*nF), ub(3*nF);
    ConstaintM = Eigen::MatrixXd::Zero(4*nF, 3*nF);
    ConstaintM.block<4,3>(0,0) << (Eigen::MatrixXd(4,3) << 1, 0,  mu,
                                                                                                                  0, 1, mu,
                                                                                                                  1, 0, -mu,
                                                                                                                  0, 1, -mu).finished();
    lbA.head(4) << -INFTY, -INFTY, 0, 0;
    ubA.head(4) << 0, 0, INFTY, INFTY;
    lb.head(3) << -INFTY,-INFTY, fz_low;
    ub.head(3) << INFTY,INFTY,  fz_high;

    for(int i = 1; i <= (nF-1); i++)
    {
        ConstaintM.block<4,3>(4*i, 3*i) << ConstaintM.block<4,3>(0, 0);
        lbA.segment(i*4, 4) << lbA.segment(0, 4);
        ubA.segment(i*4, 4) << ubA.segment(0, 4);
        lb.segment(i*3, 3) << lb.segment(0, 3);
        ub.segment(i*3, 3) << ub.segment(0, 3);
    }
    for(int i = 0; i < nF; i++)
    {
        if(supporting[i] == 0)
        {
             lb.segment(i*3, 3) << 0, 0, 0;
             ub.segment(i*3, 3) << 0, 0, 0;
        }
    }
    QProblem QP_MPC( 3*nF, 4*nF);
    QP_MPC.setPrintLevel(PL_LOW);
    int_t nWSR = 100;
    double tic = getCPUtime();
    QP_MPC.init( Hm.data(), gv.data(), ConstaintM.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR );
    double toc = getCPUtime();
//    std::cout << "MPC solutin time: " << toc-tic << std::endl;
    double* psolution = new double[3*nF];
    memset(psolution, 0, sizeof(double)*3*nF);
    QP_MPC.getPrimalSolution( psolution);
//    std::cout << "MPC solution is :" << std::endl;
    for(int i = 0; i < 3*nF; i++)
    {
         solution[i] = psolution[i];
    }
//    std::cout << solution << std::endl;
//    std::cout << "ObjVal = " << QP_MPC.getObjVal()+(A*X0-Xref).transpose()*L*(A*X0-Xref) << std::endl;
//    std::cout << "Ax+Bu = " << std::endl << A*X0+B*solution << std::endl;
    delete [] psolution;
    delete [] m;
}

void MPCController::SolveAddPIDCompensation(VectorXd X0, Vector3d rn[])
{
///////////////////////////////////
//    ktau = 3e3;
//    kF = 3e3;
//    alpha_regularization = 1e-2;
//    dF = 3e2;
//    dtau = 3e2;
 //  These parameters acheives good result with 100Hz control frequency.
////////////////////////////////////


    for(int i = 0; i < nF; i++)
    {
        //Setting about whether in support or not
        if(supporting[i] == 0)
        {
            rn[i] << 0, 0, 0;
        }
    }

//    std::cout << "X0" << std::endl << X0 << std::endl;
//    std::cout << "Xref" << std::endl << Xref << std::endl;
//    cout << "kF:" << endl << kF << endl << "dF:" << endl << dF << endl << "Tau:" << endl << ktau << endl<< "dtau:" << endl << dtau << endl ;
    Vector3d F = kF*(Xref.segment(3,3) - X0.segment(3,3)) + dF*(Xref.segment(9,3) - X0.segment(9,3)),
                       Tau = ktau*( Xref.head(3) - X0.head(3) ) + dtau*(Xref.segment(6,3) - X0.segment(6,3));
    //!!!!!!!!!!!!!!!!!!!!! We can use this information to decide whether to move !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    Vector3d low(-1000, -500, -1500), high(1000, 500, 500);
    F = MyMathFunc::ClipEachElementInVector(F, low, high);
    low << -400, -400, -400;
    high << 400, 400, 400;
    Tau = MyMathFunc::ClipEachElementInVector(Tau, low, high);
 //   std::cout << "PID F: " << std::endl << F << std::endl << "PID Tau: " << Tau << std::endl;
    // Force distribution
    MatrixXd P(3, 3*nF), alphaF(3*nF, 3*nF), alphaTau(3*nF, 3*nF);
    alphaF = MatrixXd::Identity(3*nF, 3*nF)*alphaF_regularization;
    alphaTau = MatrixXd::Identity(3*nF, 3*nF)*alphaTau_regularization;

    for(int i = 0; i < nF; i++)
    {
        P.block<3,3>(0, i*3) = MyMathFunc::CrossProductMatrixFromVector(rn[i]);
    }
//    std::cout << "rn : " <<  std::endl << rn[0] << rn[1] << rn[2] << rn[3]<< std::endl;
//    std::cout << "Solve and compensate, matrix P: " <<  std::endl << P << std::endl;
    USING_NAMESPACE_QPOASES
            double tic = getCPUtime();
    QProblem QP_Force(3*nF, 3), QP_Torque(3*nF, 3);
    real_t maxCPUtime = 0.005;
    int nWSR = 1e9;
    QP_Force.setPrintLevel(PL_LOW);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor> H(3*nF, 3*nF);
    Eigen::RowVectorXd g(3*nF);
    g = VectorXd::Zero(3*nF);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ConstaintM(3, 3*nF);
    Eigen::VectorXd lbA(3), ubA(3), lb(3*nF), ub(3*nF);

    lbA.head(3) << 0, 0, 0;
    ubA.head(3) << 0, 0, 0;

    for(int i = 0; i < nF; i++)
    {
        ConstaintM.block<3,3>(0, 3*i) = Eigen::Matrix3d::Identity();
        lb.segment(i*3, 3) <<  -300, -300, -700;
        ub.segment(i*3, 3) <<  300,  300,   300;
        //Setting about whether support or not
        if(supporting[i] == 0)
        {
             lb.segment(i*3, 3) << 0, 0, 0;
             ub.segment(i*3, 3) << 0, 0, 0;
        }
    }

    // Force distribution
    returnValue rV;
    H.noalias() = 2*(ConstaintM.transpose()*ConstaintM+alphaF);
    g = -2*(ConstaintM.transpose()*F);
    rV = QP_Force.init(H.data(), g.data(), P.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR, &maxCPUtime);
    double* pFsol = new double[3*nF];
    memset(pFsol, 0, sizeof(double)*3*nF);
    if(rV == SUCCESSFUL_RETURN)
        QP_Force.getPrimalSolution(pFsol);

    Eigen::VectorXd printmsg(3*nF);
    for(int i = 0; i < 3*nF; i++)
    {
        printmsg[i] = pFsol[i];
    }
//    std::cout << "SolveAddCompensation : Position " << std::endl << printmsg << std::endl;

    // Torque distribution
    QP_Torque.setPrintLevel(PL_LOW);
    H.noalias() = 2*(P.transpose()*P+alphaTau);
    g = -2*(P.transpose()*Tau);

    //std::cout << "lb: " << std::endl << lb << std::endl << "ub: " << ub <<std::endl;
     rV = QP_Torque.init(H.data(), g.data(), ConstaintM.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR, &maxCPUtime);

    double* pTausol = new double[3*nF];
    memset(pTausol, 0, sizeof(double)*3*nF);
    if(rV == SUCCESSFUL_RETURN)
        QP_Torque.getPrimalSolution(pTausol);
    for(int i = 0; i < 3*nF; i++)
    {
        printmsg[i] = pTausol[i];
    }
//    std::cout << "SolveAddCompensation : Attitude " << std::endl << printmsg << std::endl;

    // Get original MPC solution and attain the final solution.
    Solve(X0, rn);
    solution = GetSolution();
    solution_UniformForceMPC = solution;

//    std::cout << "Original MPC solution = " << std::endl << solution << std::endl;

    for(int i = 0; i < 3*nF; i++)
    {
        solution_PDCompensator[i] = pFsol[i] + pTausol[i];
        solution[i] += solution_PDCompensator[i];
        // Clip. Keep friction cone and z direction <=0
        if( (i+1) % 3 == 0)
        {
            solution[i] = min(0.0, solution[i]);
            solution[i-2] = MyMathFunc::sgn(solution[i-2]) * min( abs(solution[i-2]), abs(mu*solution[i]) );
            solution[i-1] = MyMathFunc::sgn(solution[i-1]) * min( abs(solution[i-1]), abs(mu*solution[i]) );
        }
    }
//    std::cout << "Final solution = " << std::endl << solution << std::endl;
    double toc = getCPUtime();
//    std::cout << "Time consume = " << std::endl << toc-tic << std::endl;
    delete [] pFsol;
    delete [] pTausol;
}

VectorXd MPCController::GetSolution()
{
    return solution;
}

VectorXd MPCController::GetSolutionUniformForceMPC()
{
    return solution_UniformForceMPC;
}

VectorXd MPCController::GetSolutionPDCompensator()
{
    return solution_PDCompensator;
}

void MPCController::CalculateDCM(Vector3d angle)
{
    Matrix3d alpha_rolln, alpha_pitchn, alpha_yawn;
    alpha_rolln << 1,            0,                      0,
                                 0,  cos(angle.x()),  -sin(angle.x()),
                                 0,  sin(angle.x()) ,  cos(angle.x());

    alpha_pitchn << cos(angle.y()), 0, sin(angle.y()),
                                                0,             1,           0       ,
                                    -sin(angle.y()), 0, cos(angle.y());

    alpha_yawn << cos(angle.z()), -sin(angle.z()),  0,
                                  sin(angle.z()),  cos(angle.z()),  0,
                                          0,                   0,                        1;
    BodytoWorldDCM.noalias() = alpha_yawn*alpha_pitchn*alpha_rolln;
}

void MPCController::SetXref(const VectorXd& xref)
{
    Xref = xref;
}

VectorXd MPCController::GetXref()
{
    return Xref;
}

void MPCController::ResetPIDCompensationParameters()
{
    // Initial stand parameters value
    // kF = 6e3*Matrix3d::Identity();
//    dF = 8e2*Matrix3d::Identity();
//    ktau = 5e3*Matrix3d::Identity();
//    dtau = 5e2*Matrix3d::Identity();
//    alphaF_regularization = 1e-3;
//    alphaTau_regularization = 1e-3;
    kF = 3e3*Matrix3d::Identity();
    dF = 3e2*Matrix3d::Identity();
    ktau = 3e3*Matrix3d::Identity();
    dtau = 3e2*Matrix3d::Identity();
    alphaF_regularization = 1e-3;
    alphaTau_regularization = 5e-4;
}
void MPCController::SetMode(MODE mode)
{
    ResetPIDCompensationParameters();
    switch (mode)
    {
    case MODE::AttitudePosition:
        attitude_weights << 50, 50, 50;
        position_weights << 50, 50, 50;
        anglesp_weights << 0, 0, 0;
        vel_weights << 0, 0, 0;
        break;

    case MODE::AttitudeVelocity:
/* Keep values strictly in attitude roll, pitch. Loose controlling in z and yaw
 * Parameters:
 * attitude_weights << 50, 50, 10;
        position_weights << 0, 0, 10;
        anglesp_weights << 0, 0, 0;
        vel_weights << 50, 50, 0;
        kF(0,0) = 0;
        kF(1,1) = 0;
        dF(0,0) = 2e3;
        dF(1,1) = 2e3;

        kF(2,2) = 1e3; dF(2,2) = 1e2;
        ktau(2,2) = 1e2; dtau(2,2) = 1e1;
*/
        attitude_weights << 50, 50, 50;
        position_weights << 0, 0, 10;
        anglesp_weights << 0, 0, 0;
        vel_weights << 50, 50, 0;
        kF(0,0) = 0;
        kF(1,1) = 0;
        dF(0,0) = 800;
        dF(1,1) = 800;

        break;
    case MODE::PositionAnglespeed:
        attitude_weights << 0, 0, 0;
        position_weights << 50, 50, 50;
        anglesp_weights << 50, 50, 50;
        vel_weights << 0, 0, 0;
        break;

    case MODE::VelocityAnglespeed:
        attitude_weights << 50, 50, 0;
        position_weights << 0, 0, 50;
        anglesp_weights << 0, 0, 50;
        vel_weights << 50, 50, 0;
        kF(0,0) = 0;
        kF(1,1) = 0;
        dF(0,0) = 5e2;
        dF(1,1) = 5e2;
        ktau(2, 2) = 0;
        dtau(2, 2) = 1e3;
        break;
    case MODE::WALK:
        attitude_weights << 50, 50, 50;
        position_weights << 1, 1, 50;
        anglesp_weights << 1, 1, 1;
        vel_weights << 50, 50, 50;
        kF(0,0) = 0;
        kF(1,1) = 0;
        dF(0,0) = 2e3;
        dF(1,1) = 2e3;

        break;
    case MODE::JUMP:
        attitude_weights << 50, 50, 50;
        position_weights << 50, 50, 0;
        anglesp_weights << 0, 0, 0;
        vel_weights << 50, 50, 50;
        kF(0,0) = 0;
        kF(1,1) = 0;
        kF(2,2) = 0;
        dF(0,0) = 2e3;
        dF(1,1) = 2e3;
        dF(2,2) = 2e3;
        break;
    case MODE::BOUND:
        attitude_weights << 50, 50, 50;
        position_weights << 50, 50, 1;
        anglesp_weights << 1, 1, 1;
        vel_weights << 50, 50, 50;
        kF(0,0) = 0;
        kF(1,1) = 0;
        kF(2,2) = 0;
        dF(0,0) = 2e3;
        dF(1,1) = 2e3;
        dF(2,2) = 1e3;

        break;
    }
    L.block<3,3>(0, 0) = attitude_weights.asDiagonal();
    L.block<3,3>(3, 3) = position_weights.asDiagonal();
    L.block<3,3>(6, 6) = anglesp_weights.asDiagonal();
    L.block<3,3>(9, 9) = vel_weights.asDiagonal();
}


void Debug()
{
    //    cout << "r1 " << endl << pnF[0] <<endl;
    //    cout << "r2 " << endl << pnF[1] <<endl;
    //    cout << "r3 " << endl << pnF[2] <<endl;
    //    cout << "r4 " << endl << pnF[3] <<endl;
    //    cout << "A" << endl << A << endl;
    //    cout << "B" << endl << B << endl;
    //    cout << " H " << endl <<Hm <<endl;
    //    cout << " gv " << endl << gv <<endl;
    //    cout << " Constrain Matrix  " << endl << ConstaintM <<endl;
    //    cout << " lbA " << endl << lbA <<endl;
    //    cout << " ubA " << endl << ubA <<endl;
}

