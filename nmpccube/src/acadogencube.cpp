#include <acado_code_generation.hpp>
#include <iostream>  

USING_NAMESPACE_ACADO

int main()
{
    //
    // Variables
    //
    const double m = 5.0, g = 9.80;
    const double Ixx = 0.25, Iyy = 0.25, Izz = 0.325;
    double Kdx = 0.0000267;
    double Kdy = 0.0000267;
    double Kdz = 0.0000625;

    DifferentialState x, y, z, phi, theta, psi;
    DifferentialState xd, yd, zd, phid, thetad, psid;
    Control fy, fz, c1, c2, c3;

    DifferentialEquation f;
    f << dot(x) == xd;
    f << dot(y) == yd;
    f << dot(z) == zd;
    f << dot(phi) == phid;
    f << dot(theta) == thetad;
    f << dot(psi) == psid;
    // Jr = 0; 
    f << dot(xd) == (1/m) * ((- sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi)) * fy + (sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi)) * fz - Kdx * xd);
    f << dot(yd) == (1/m) * ((cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi)) * fy + (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) * fz - Kdy * yd);
    f << dot(zd) == (1/m) * (cos(theta) * cos(phi) * fz - m * g - Kdz * zd);
    f << dot(phid) ==   (1/Ixx) * ( phid * thetad * (Iyy - Izz) + c1);
    f << dot(thetad) == (1/Iyy) * ( phid * psid * (Izz - Ixx) + c2);
    f << dot(psid) == (1/Izz) * ( phid * thetad * (Ixx - Iyy) + c3);

    Function rf;
    Function rfN;

    rf << z << phi << theta << psi;
    rf << fy << fz << c1 << c2 << c3;

    rfN << z  << phi << theta << psi;

    DMatrix W = eye<double>(rf.getDim());
    DMatrix WN = eye<double>(rfN.getDim());

    W(0, 0) = 12.75;
    W(1, 1) = 7.5;
    W(2, 2) = 7.5;
    W(3, 3) = 7.5;
    W(4, 4) = 7.75;
    W(5, 5) = 70.75;
    W(6, 6) = 70.75;
    W(7, 7) = 70.75;
    W(7, 7) = 70.75;

    WN(0, 0) = 0.5;
    WN(1, 1) = 0.5;
    WN(2, 2) = 0.5;
    WN(3, 3) = 0.5;

    //
    // Optimal Control Problem
    //
    const int N = 12;
    const int Ni = 5;
    const double Ts = 0.05;

    OCP ocp(0, N * Ts, N);
    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);
    ocp.subjectTo( 0 <= fy <= 10);  
    ocp.subjectTo( 40 <= fz <= 60);  
    ocp.subjectTo(-2.5 <= c1 <= 2.5);  
    ocp.subjectTo(-2.5 <= c2 <= 2.5);  
    ocp.subjectTo(-2.5 <= c3 <= 2.5);  

    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

    mpc.set(INTEGRATOR_TYPE, INT_RK45);
	mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);

    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(HOTSTART_QP, YES);

    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
	

    if (mpc.exportCode("cube_gen") != SUCCESSFUL_RETURN)
        exit(EXIT_FAILURE);

    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}
