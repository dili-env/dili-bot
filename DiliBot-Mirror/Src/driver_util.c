/// @file driver_util.c
/// @brief driver utility and simple control theory function declaration
#include "driver_util.h"
#include "driver.h"

#define rW      0.029
#define rK      0.1225
#define T       0.005f
#define PI      3.141592f

//#define MAGIC   66.66666f             // 800(duty)/12(V)

#define MAGIC   1200


#define K_thetax        -40.9529
#define K_thetax_dot    -2.1880

#define K_thetay        -40.9531
#define K_thetay_dot    -2.1890

#define K_phix          -0.3976
#define K_phix_dot      -0.8529 

#define K_phiy          -0.3977
#define K_phiy_dot      -0.8521



#define ENCODER_PULSE_1_CYCLE   1452.0f
extern uint32_t test_count;
extern float imu_value_rad[3];
extern float pitch_y, raw_x, yaw_z;

float get_phix_dot(float theta_x_dot, float psi1_dot, float psi2_dot, float psi3_dot);
float get_phiy_dot(float theta_y_dot, float psi1_dot, float psi2_dot, float psi3_dot);

float get_phix(float psi1, float psi2, float psi3);
float get_phiy(float psi1, float psi2, float psi3);

float get_phix_dot(float theta_x_dot, float psi1_dot, float psi2_dot, float psi3_dot) {
  // Note: sqrt(6) = 2.4495, sqrt(2) = 1.4142
  // imu_value_rad[1], imu_value_rad[2] unit is radian
  float tmp = (1/(3*rK))*( 2.4495*rW*sin(imu_value_rad[2])*sin(imu_value_rad[1])*(-psi2_dot + psi3_dot) + \
                           1.4142*rW*cos(imu_value_rad[2])*sin(imu_value_rad[1])*(psi1_dot + psi2_dot + psi3_dot) + \
                           cos(imu_value_rad[1])*(1.4142*rW*(-2*psi1_dot + psi2_dot + psi3_dot) + 3*rK*theta_x_dot) );
  return tmp;
}

float get_phiy_dot(float theta_y_dot, float psi1_dot, float psi2_dot, float psi3_dot) {
    //
    //
  float tmp = (1/(3*rK))*( 2.4495*rW*cos(imu_value_rad[2])*(-psi2_dot + psi3_dot) - \
                           1.4142*rW*sin(imu_value_rad[2])*(psi1_dot + psi2_dot + psi3_dot) +\
                           3*rK*theta_y_dot);
  return tmp;
}


float get_phix(float psi1, float psi2, float psi3) {
  return 0.9428f*psi1 - 0.4714f*psi2 - 0.4714f*psi3;
}

float get_phiy(float psi1, float psi2, float psi3) {
    return -0.8165f*psi2 + 0.8165f*psi3;
}



void simple_control_LQR(void) {


  // TODO: from here again
  volatile int pulse_diff;
  volatile static int pre_pulse[3] = {0, 0, 0};
  volatile float psi_dot[3];
  
  volatile static float pre_thetax = 0.0;            // [rad]
  volatile static float pre_thetay = 0.0;            // [rad]
  volatile float thetax_dot, thetay_dot;             // [rad/s]
  
  volatile static float phix = 0.0;             // [rad]
  volatile static float phiy = 0.0;             // [rad]

  volatile float phix_dot, phiy_dot;

  volatile float Tx, Ty;
  volatile float T1, T2, T3;
  

  /// Calculate psi_dot (rad/s)
//  for (int i = 0; i<3; i++) {
//    pulse_diff = encoder_ReadMotor((MotorIndex)i)- pre_pulse[i];
//    pre_pulse[i] = encoder_ReadMotor((MotorIndex)i);
//    psi_dot[i] = (((float)pulse_diff)*2.0f*PI / ENCODER_PULSE_1_CYCLE) / T; // rad/s
//  }


  /// Calculate theta_dot (rad/s)
  thetax_dot = (imu_value_rad[2] - pre_thetax) / T; // rad/s
  pre_thetax = imu_value_rad[2];                    // rad
  thetay_dot = (imu_value_rad[1] - pre_thetay) / T; // rad/s
  pre_thetay = imu_value_rad[1];                    // rad


//  // Calculate phix_dot, phiy_dot from thetax_dot, thetay_dot and psi_dot (rad/s)
//  phix_dot = get_phix_dot(thetax_dot, psi_dot[0], psi_dot[1], psi_dot[2]);  // rad/s
//  //phix_dot = get_phix(psi_dot[0], psi_dot[1], psi_dot[2]);
//  phix += phix_dot * T;                                                     // rad

//  phiy_dot = get_phiy_dot(thetay_dot, psi_dot[0], psi_dot[1], psi_dot[2]);  // rad/s
//  //phiy_dot = get_phiy(psi_dot[0], psi_dot[1], psi_dot[2]);
//  phiy += phiy_dot * T;                                                     // rad

//  // #test
//  printf("phix     = %.02f, phiy     = %.02f\r\n", phix, phiy);
//  printf("phix_dot = %.02f, phiy_dot = %.02f\r\n", phix_dot, phiy_dot);
//  printf("thetax     = %.04f, thetay = %.04f\r\n", imu_value_rad[1], imu_value_rad[2]);
//  printf("thetax_dot = %.04f, thetay_dot = %.04f\r\n", thetax_dot, thetay_dot);
//  printf("Test count: %d-----------\r\n\n", test_count);
//  // #test

  // Control law
//  Tx = -(K_phix*phix + K_thetax*imu_value_rad[2] + \
//         K_phix_dot*phix_dot + K_thetax_dot*thetax_dot) ;
//  Ty = -(K_phiy*phiy + K_thetay*imu_value_rad[1] + \
//         K_phiy_dot*phiy_dot + K_thetay_dot*thetay_dot) ;


  Tx = -(K_thetax*imu_value_rad[2] + K_thetax_dot*thetax_dot) ;
  Ty = -(K_thetay*imu_value_rad[1] + K_thetay_dot*thetay_dot) ;


//  // #test
//  printf("Tx = %d, Ty = %d\r\n", (int)Tx, (int)Ty);
//  printf("Test count: %d-----------\r\n\n", test_count);
//  // #test

  // Note * 200 for contrl
//  T1 = (20)*(Tx - Ty);
//  T2 = (10)*(-2.7321*Tx - 0.7321*Ty);
//  T3 = (10)*(0.7321*Tx + 2*Ty);

  T1 = (0.9428f)*(Tx)              * MAGIC;
  T2 = (0.4714f)*(-Tx - 1.7321f*Ty) * MAGIC;
  T3 = (0.4714f)*(-Tx + 1.7321f*Ty) * MAGIC;


  // #test
//  printf("T1 = %d, T2 = %d, T3 = %d, count = %d\r\n", (int)T1, (int)T2, (int)T3, test_count);
//  printf("Test count: %d-----------\r\n\n", test_count);
  // #test

  
  motor_SetSpeed(MOTOR_1, (int)T1);
  motor_SetSpeed(MOTOR_2, (int)T2);
  motor_SetSpeed(MOTOR_3, (int)T3);
  
}


void sliding_control (void) {
  float theta[2], theta_dot[2];
  static float pre_theta[2];
  static char init_state = 0;
  
  float inv_g4, S, f, f2;
  float Txy[2], T1, T2, T3;
  
  int i = 0;
  
  float K_sat;
  
  uint8_t buff_count = (uint8_t)test_count;
  
  // dma_printf(&buff_count, 1);
  
  // Theta[0] = thetax
  // Theta[1] = thetay
  theta[1] = raw_x;
  theta[0] = pitch_y;
  
  //theta[0] = imu_value_rad[2];
  //theta[1] = imu_value_rad[1];
  
  if (init_state == 0) {
    init_state = 1;
    pre_theta[0] = theta[0];
    pre_theta[1] = theta[1];
  }
  
  for (i = 0; i < 2; i++) {
    theta_dot[i] = (theta[i] - pre_theta[i]) / T;
    pre_theta[i] = theta[i];
    
    inv_g4 = (cos(theta[i])-0.7693f*(cos(theta[i]))*(cos(theta[i]))+2.3967f) / 
             (-19.2152f-36.2312f*cos(theta[i]));
    
    K_sat = 0.2f;
    
    S = theta[i] + 0.5f*theta_dot[i];

    if (S > K_sat)
      S = K_sat;
    else if (S < -K_sat)
      S = -K_sat;
    
    // S = S * 10.0f;
    
    f = 0.5f*theta_dot[i];
    
    f2 = (72.7425f*sin(theta[i]) - 0.7693f*cos(theta[i])*sin(theta[i])*theta_dot[i]*theta_dot[i] + 0.5f*sin(theta[i])*theta_dot[i]*theta_dot[i] ) / 
         (cos(theta[i]) - 0.7693f*cos(theta[i])*cos(theta[i]) + 2.3967f);
    
    Txy[i] = -inv_g4*(S + f + f2) * MAGIC;
  
  }
  
  T1 = (0.9428f)*( Txy[0])                  ;
  T2 = (0.4714f)*(-Txy[0] - 1.7321f*Txy[1]) ;
  T3 = (0.4714f)*(-Txy[0] + 1.7321f*Txy[1]) ;
  
  
  //printf("T1 = %d, T2 = %d, T3 = %d\r\n", (int)T1, (int)T2, (int)T3);
  
  motor_SetSpeed(MOTOR_1, (int)T1);
  motor_SetSpeed(MOTOR_2, (int)T2);
  motor_SetSpeed(MOTOR_3, (int)T3);
}




void simple_control_LQR_2(void) {
  float psi[3];                            // [xung/number of pulse]
  static float pre_thetax = 0.0;           // [rad]
  static float pre_thetay = 0.0;           // [rad]
  static float pre_phix = 0.0;             // [rad]
  static float pre_phiy = 0.0;             // [rad]

  float psi_dot[3];
  float thetax_dot, thetay_dot;
  float phix, phiy, phix_dot, phiy_dot;

  double Tx, Ty;
  double T1, T2, T3;
  
  int T1_int, T2_int, T3_int;


  // Calculate phi (rad/s):
  for (int i = 0; i < 3; i++) {
    // psi (1,2,3) = encoder * 2*pi / num_of_pulse
    psi[i] =  (float)encoder_ReadMotor((MotorIndex)i) * 2.0f * PI / ENCODER_PULSE_1_CYCLE;
  }
  phix = get_phix(psi[0], psi[1], psi[2]);
  phiy = get_phiy(psi[0], psi[1], psi[2]);

  // Calculate phi_dot
  phix_dot = (phix - pre_phix) / T;
  pre_phix = phix;
  phiy_dot = (phiy - pre_phiy) / T;
  pre_phiy = phiy;
  
  // Calculate theta_dot (rad/s):
  thetax_dot = (imu_value_rad[1] - pre_thetax) / T; // rad/s
  pre_thetax = imu_value_rad[1];                    // rad
  thetay_dot = (imu_value_rad[2] - pre_thetay) / T; // rad/s
  pre_thetay = imu_value_rad[2];                    // rad

//  // #test
//  printf("phix     = %.02f, phiy     = %.02f\r\n", phix, phiy);
//  printf("phix_dot = %.02f, phiy_dot = %.02f\r\n", phix_dot, phiy_dot);
//  printf("thetax     = %.04f, thetay = %.04f\r\n", imu_value_rad[1], imu_value_rad[2]);
//  printf("thetax_dot = %.04f, thetay_dot = %.04f\r\n", thetax_dot, thetay_dot);
  
  
//  printf("System State: thetax = %.04f, thetax_dot = %.04f, thetay = %.04f, thetay_dot = %.04f\r\n", imu_value_rad[1], thetax_dot, imu_value_rad[2], thetay_dot);
//  printf("Test count: %d-----------\r\n\n", test_count);
//  // #test

  // Control law
  Tx = -(K_phix*phix + K_thetax*imu_value_rad[1] + \
         K_phix_dot*phix_dot + K_thetax_dot*thetax_dot);
  Ty = -(K_phiy*phiy + K_thetay*imu_value_rad[2] + \
         K_phiy_dot*phiy_dot + K_thetay_dot*thetay_dot);

//  // #test
//  printf("Tx = %d, Ty = %d\r\n", (int)Tx, (int)Ty);
//  printf("Test count: %d-----------\r\n\n", test_count);
//  // #test

  // Note * 200 for contrl
//  T1 = (20)*(Tx - Ty);
//  T2 = (10)*(-2.7321*Tx - 0.7321*Ty);
//  T3 = (10)*(0.7321*Tx + 2*Ty);

  T1 = (0.9428)*(Tx)              * MAGIC;
  T2 = (0.4714)*(-Tx - 1.7321*Ty) * MAGIC;
  T3 = (0.4714)*(-Tx + 1.7321*Ty) * MAGIC;


  // #test
  printf("T1 = %d, T2 = %d, T3 = %d\r\n", (int)T1, (int)T2, (int)T3);
//  printf("Test count: %d-----------\r\n\n", test_count);
  // #test
  
//  T1_int = (int)(T1*700.0/(T1+T2+T3));
//  T2_int = (int)(T2*700.0/(T1+T2+T3));
//  T3_int = (int)(T3*700.0/(T1+T2+T3));
//  
  motor_SetSpeed(MOTOR_1, (int)T1);
  motor_SetSpeed(MOTOR_2, (int)T2);
  motor_SetSpeed(MOTOR_3, (int)T3);
  
}
