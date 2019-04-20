/// @file encoder_util.c
/// @brief This file provice equation to calculate control parameter from real encoder value

// Note: The equation in this file is refer from Odemetry mudule of ETH

// TODO: if we have rK, rW let calculate as much as posible

#include <math.h>

#define rW      1
#define rK      1
#define T       0.005
#define PI      3.14

#define ENCODER_PULSE_1_CYCLE   1452

/*
 * Input:   psi1, psi2, psi3, thetax, thetay
 * Output:  phix, phix_dot, phiy, phiy_dot
*/


extern double thetax, thetay;   // [rad]


int get_encoder_value(int encoder_index);

double get_phix_dot(float theta_x, float theta_y, float theta_x_dot, float psi1_dot, float psi2_dot, float psi3_dot) {
    // Note: sqrt(6) = 2.4495, sqrt(2) = 1.4142
    // theta_x, theta_y unit is radian
    return 1/(3*rK) * (  2.4495*rW*sin(theta_x)*sin(theta_y)*(-psi2_dot + psi3_dot) + \
                         1.4142*rW*cos(theta_x)*sin(theta_y)*(psi1_dot + psi2_dot + psi3_dot) + \
                         cos(theta_y)*(1.4142*rW*(-2*psi1_dot + psi2_dot + psi3_dot) + 3*rK*theta_x_dot) );
}

double get_phiy_dot(float theta_x, float theta_y, float theta_y_dot, float psi1_dot, float psi2_dot, float psi3_dot) {
    //
    //
    return 1/(3*rK) * ( 2.4495*rW*cos(theta_x)*(-psi2_dot + psi3_dot) - \
                        1.4142*rW*sin(theta_x)*(psi1_dot + psi2_dot + psi3_dot) +\
                        3*rK*theta_y_dot);
}


void control_trial () {
    static int pre_psi[3] = {0, 0, 0};      // [xung/number of pulse]
    static double pre_thetax = 0;           // [rad]
    static double pre_thetay = 0;           // [rad]
    static double pre_phix = 0;             // [rad]
    static double pre_phiy = 0;             // [rad]

    double psi_dot[3];
    double thetax_dot, thetay_dot;

    double phix, phiy, phix_dot, phiy_dot;

    int i = 0;

    // Calculate psi_dot (rad/s):
    for (i = 0; i < 3; i++) {
        psi_dot[i] = ((get_encoder_value(i) - pre_psi[i]) * 2 * PI / ENCODER_PULSE_1_CYCLE) / T;
        pre_psi[i] = get_encoder_value(i);
    }

    // Calculate theta_dot (rad/s):
    thetax_dot = (thetax - pre_thetax) / T; // rad/s
    pre_thetax = thetax;                    // rad
    thetay_dot = (thetay - pre_thetay) / T; // rad/s
    pre_thetay = thetay;                    // rad


    // Calculate phix, phiy from phix_dot, phiy_dot;
    phix_dot = get_phix_dot(thetax, thetay, thetax_dot, psi_dot[0], psi_dot[1], psi_dot[2]);    // rad/s
    phix = phix_dot * T + pre_phix;     // rad
    pre_phix = phix;                    // rad

    phiy_dot = get_phix_dot(thetax, thetay, thetay_dot, psi_dot[0], psi_dot[1], psi_dot[2]);    // rad/s
    phix = phix_dot * T + pre_phix;     // rad
    pre_phix = phix;                    // rad



    // OK now we have phix, phix_dot, phiy, phiy_dot, thetax, thetax_dot, thetay, thetay_dot
    // Apply control theory here we can calculate the T1, T2, T3
}
