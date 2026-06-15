/* This file has been generated from /workspace/paparazzi/conf/airframes/ENAC/quadrotor/anton_mfc.xml */
/* Version v7.0_unstable-339-g66c560591-dirty */
/* Please DO NOT EDIT */

#ifndef AIRFRAME_H
#define AIRFRAME_H

#define AIRFRAME_NAME "ANTON_MFC"
#define AC_ID 218
#define MD5SUM ((uint8_t*)"\345\165\364\267\171\122\330\356\215\106\013\007\363\140\270\210")

#define SERVOS_DSHOT_NB 5
#include "modules/actuators/actuators_dshot.h"

#define SERVO_FR_DRIVER_NO 0
#define SERVO_FR_DRIVER_IDX 3
#define SERVO_FR_IDX 0
#define SERVO_FR_NEUTRAL 100
#define SERVO_FR_TRAVEL_UP 0.197916666667
#define SERVO_FR_TRAVEL_DOWN 0.0104166666667
#define SERVO_FR_MAX 2000
#define SERVO_FR_MIN 0

#define SERVO_BR_DRIVER_NO 0
#define SERVO_BR_DRIVER_IDX 4
#define SERVO_BR_IDX 1
#define SERVO_BR_NEUTRAL 100
#define SERVO_BR_TRAVEL_UP 0.197916666667
#define SERVO_BR_TRAVEL_DOWN 0.0104166666667
#define SERVO_BR_MAX 2000
#define SERVO_BR_MIN 0

#define SERVO_BL_DRIVER_NO 0
#define SERVO_BL_DRIVER_IDX 2
#define SERVO_BL_IDX 2
#define SERVO_BL_NEUTRAL 100
#define SERVO_BL_TRAVEL_UP 0.197916666667
#define SERVO_BL_TRAVEL_DOWN 0.0104166666667
#define SERVO_BL_MAX 2000
#define SERVO_BL_MIN 0

#define SERVO_FL_DRIVER_NO 0
#define SERVO_FL_DRIVER_IDX 1
#define SERVO_FL_IDX 3
#define SERVO_FL_NEUTRAL 100
#define SERVO_FL_TRAVEL_UP 0.197916666667
#define SERVO_FL_TRAVEL_DOWN 0.0104166666667
#define SERVO_FL_MAX 2000
#define SERVO_FL_MIN 0

static inline int get_servo_min_DSHOT(int _idx) {
  switch (_idx) {
    case SERVO_FR_DRIVER_IDX: return SERVO_FR_MIN;
    case SERVO_BR_DRIVER_IDX: return SERVO_BR_MIN;
    case SERVO_BL_DRIVER_IDX: return SERVO_BL_MIN;
    case SERVO_FL_DRIVER_IDX: return SERVO_FL_MIN;
    default: return 0;
  };
}

static inline int get_servo_max_DSHOT(int _idx) {
  switch (_idx) {
    case SERVO_FR_DRIVER_IDX: return SERVO_FR_MAX;
    case SERVO_BR_DRIVER_IDX: return SERVO_BR_MAX;
    case SERVO_BL_DRIVER_IDX: return SERVO_BL_MAX;
    case SERVO_FL_DRIVER_IDX: return SERVO_FL_MAX;
    default: return 0;
  };
}

static inline int get_servo_idx_DSHOT(int _idx) {
  switch (_idx) {
    case SERVO_FR_DRIVER_IDX: return SERVO_FR_IDX;
    case SERVO_BR_DRIVER_IDX: return SERVO_BR_IDX;
    case SERVO_BL_DRIVER_IDX: return SERVO_BL_IDX;
    case SERVO_FL_DRIVER_IDX: return SERVO_FL_IDX;
    default: return 0;
  };
}


#define SERVOS_PWM_NB 2
#include "modules/actuators/actuators_pwm.h"

#define SERVO_SWITCH_DRIVER_NO 1
#define SERVO_SWITCH_DRIVER_IDX 1
#define SERVO_SWITCH_IDX 4
#define SERVO_SWITCH_NEUTRAL 1150
#define SERVO_SWITCH_TRAVEL_UP 0.0677083333333
#define SERVO_SWITCH_TRAVEL_DOWN 0
#define SERVO_SWITCH_MAX 1800
#define SERVO_SWITCH_MIN 1150

static inline int get_servo_min_PWM(int _idx) {
  switch (_idx) {
    case SERVO_SWITCH_DRIVER_IDX: return SERVO_SWITCH_MIN;
    default: return 0;
  };
}

static inline int get_servo_max_PWM(int _idx) {
  switch (_idx) {
    case SERVO_SWITCH_DRIVER_IDX: return SERVO_SWITCH_MAX;
    default: return 0;
  };
}

static inline int get_servo_idx_PWM(int _idx) {
  switch (_idx) {
    case SERVO_SWITCH_DRIVER_IDX: return SERVO_SWITCH_IDX;
    default: return 0;
  };
}


#define COMMAND_FR 0
#define COMMAND_BR 1
#define COMMAND_BL 2
#define COMMAND_FL 3
#define COMMAND_THRUST 4
#define COMMANDS_NB_REAL 5
#define COMMANDS_NB 5
#define COMMAND_NAMES { "FR", "BR", "BL", "FL", "THRUST" }

#define COMMANDS_FAILSAFE {-9600,-9600,-9600,-9600,0}


#define ACTUATORS_CONFIG { \
    { \
      .pprz_val = 0, \
      .driver_val = 0, \
      .config = { \
        .driver_no = SERVO_FR_DRIVER_NO, \
        .servo_idx = SERVO_FR_DRIVER_IDX, \
        .min = SERVO_FR_MIN, \
        .max = SERVO_FR_MAX, \
        .neutral = SERVO_FR_NEUTRAL, \
        .travel_up = SERVO_FR_TRAVEL_UP, \
        .travel_down = SERVO_FR_TRAVEL_DOWN \
      }, \
      .set = ActuatorDShotSet \
    }, \
    { \
      .pprz_val = 0, \
      .driver_val = 0, \
      .config = { \
        .driver_no = SERVO_BR_DRIVER_NO, \
        .servo_idx = SERVO_BR_DRIVER_IDX, \
        .min = SERVO_BR_MIN, \
        .max = SERVO_BR_MAX, \
        .neutral = SERVO_BR_NEUTRAL, \
        .travel_up = SERVO_BR_TRAVEL_UP, \
        .travel_down = SERVO_BR_TRAVEL_DOWN \
      }, \
      .set = ActuatorDShotSet \
    }, \
    { \
      .pprz_val = 0, \
      .driver_val = 0, \
      .config = { \
        .driver_no = SERVO_BL_DRIVER_NO, \
        .servo_idx = SERVO_BL_DRIVER_IDX, \
        .min = SERVO_BL_MIN, \
        .max = SERVO_BL_MAX, \
        .neutral = SERVO_BL_NEUTRAL, \
        .travel_up = SERVO_BL_TRAVEL_UP, \
        .travel_down = SERVO_BL_TRAVEL_DOWN \
      }, \
      .set = ActuatorDShotSet \
    }, \
    { \
      .pprz_val = 0, \
      .driver_val = 0, \
      .config = { \
        .driver_no = SERVO_FL_DRIVER_NO, \
        .servo_idx = SERVO_FL_DRIVER_IDX, \
        .min = SERVO_FL_MIN, \
        .max = SERVO_FL_MAX, \
        .neutral = SERVO_FL_NEUTRAL, \
        .travel_up = SERVO_FL_TRAVEL_UP, \
        .travel_down = SERVO_FL_TRAVEL_DOWN \
      }, \
      .set = ActuatorDShotSet \
    }, \
    { \
      .pprz_val = 0, \
      .driver_val = 0, \
      .config = { \
        .driver_no = SERVO_SWITCH_DRIVER_NO, \
        .servo_idx = SERVO_SWITCH_DRIVER_IDX, \
        .min = SERVO_SWITCH_MIN, \
        .max = SERVO_SWITCH_MAX, \
        .neutral = SERVO_SWITCH_NEUTRAL, \
        .travel_up = SERVO_SWITCH_TRAVEL_UP, \
        .travel_down = SERVO_SWITCH_TRAVEL_DOWN \
      }, \
      .set = ActuatorPwmSet \
    }\
  }

#define ACTUATORS_NB 5

#define AllActuatorsInit() { \
  ActuatorsPwmInit();\
  ActuatorsDShotInit();\
}

#define AllActuatorsCommit() { \
  ActuatorsPwmCommit();\
  ActuatorsDShotCommit();\
}

#define SetActuatorsFromCommands(values, AP_MODE) { \
  int32_t actuator_value_pprz;\
\
  actuator_value_pprz = values[COMMAND_FR]; \
  ActuatorSet(FR, actuator_value_pprz); \
\
  actuator_value_pprz = values[COMMAND_BR]; \
  ActuatorSet(BR, actuator_value_pprz); \
\
  actuator_value_pprz = values[COMMAND_BL]; \
  ActuatorSet(BL, actuator_value_pprz); \
\
  actuator_value_pprz = values[COMMAND_FL]; \
  ActuatorSet(FL, actuator_value_pprz); \
\
  AllActuatorsCommit(); \
}

#define SECTION_IMU 1
#define IMU_GYRO_P_SIGN -1
#define IMU_GYRO_Q_SIGN -1
#define IMU_GYRO_R_SIGN 1
#define IMU_ACCEL_X_SIGN -1
#define IMU_ACCEL_Y_SIGN -1
#define IMU_ACCEL_Z_SIGN 1
#define IMU_ACCEL_X_NEUTRAL -90
#define IMU_ACCEL_Y_NEUTRAL 40
#define IMU_ACCEL_Z_NEUTRAL 12
#define IMU_ACCEL_X_SENS 2.321864827044042
#define IMU_ACCEL_X_SENS_NUM 11405
#define IMU_ACCEL_X_SENS_DEN 4912
#define IMU_ACCEL_Y_SENS 2.450207588413862
#define IMU_ACCEL_Y_SENS_NUM 43082
#define IMU_ACCEL_Y_SENS_DEN 17583
#define IMU_ACCEL_Z_SENS 2.4560049628471914
#define IMU_ACCEL_Z_SENS_NUM 63333
#define IMU_ACCEL_Z_SENS_DEN 25787
#define IMU_MAG_X_SIGN -1
#define IMU_MAG_Y_SIGN -1
#define IMU_MAG_Z_SIGN 1
#define IMU_MAG_X_NEUTRAL 437
#define IMU_MAG_Y_NEUTRAL -2357
#define IMU_MAG_Z_NEUTRAL 1978
#define IMU_MAG_X_SENS 0.6420696170457475
#define IMU_MAG_X_SENS_NUM 8153
#define IMU_MAG_X_SENS_DEN 12698
#define IMU_MAG_Y_SENS 0.6426945667319279
#define IMU_MAG_Y_SENS_NUM 17507
#define IMU_MAG_Y_SENS_DEN 27240
#define IMU_MAG_Z_SENS 0.6220200205771864
#define IMU_MAG_Z_SENS_NUM 3418
#define IMU_MAG_Z_SENS_DEN 5495
#define IMU_BODY_TO_IMU_PHI 0.
#define IMU_BODY_TO_IMU_THETA 0.
#define IMU_BODY_TO_IMU_PSI -0.7853981625

#define SECTION_MAG 1
#define LIS3MDL_MAG_TO_IMU_PHI 0.
#define LIS3MDL_MAG_TO_IMU_THETA 0.
#define LIS3MDL_MAG_TO_IMU_PSI 0.

/* XML conf/mag/toulouse_muret.xml */
#define SECTION_MAG_MODEL 1
#define INS_H_X 0.515118
#define INS_H_Y 0.0125878
#define INS_H_Z 0.857027

#define SECTION_STABILIZATION_ATTITUDE 1
#define STABILIZATION_ATTITUDE_SP_MAX_PHI 0.7853981625
#define STABILIZATION_ATTITUDE_SP_MAX_THETA 0.7853981625
#define STABILIZATION_ATTITUDE_SP_MAX_R 1.04719755
#define STABILIZATION_ATTITUDE_DEADBAND_R 250

#define SECTION_STABILIZATION_MFC 1
#define STABILIZATION_MFC_NUM_ACT 4
#define STABILIZATION_MFC_G1 { {-40, -40,  40,  40} , { 40, -40, -40,  40} , {  5,  -5,   5,  -5} , {-1.5, -1.5, -1.5, -1.5} }
#define STABILIZATION_MFC_G2 {150.0,   -150.0,  150.0,   -150.0 }
#define STABILIZATION_MFC_ACT_FREQ {30.5, 30.5, 30.5, 30.5}
#define STABILIZATION_MFC_ESTIMATION_FILT_CUTOFF 4.0
#define STABILIZATION_MFC_WLS_PRIORITIES {1000, 1000, 1, 100}
#define STABILIZATION_MFC_COMMANDS {COMMAND_FR, COMMAND_BR, COMMAND_BL, COMMAND_FL}
#define STABILIZATION_MFC_ROLL_TIME_TRAJECTORY 50.
#define STABILIZATION_MFC_ROLL_INTEGRATION_WINDOW 5.
#define STABILIZATION_MFC_ROLL_ALPHA 147.0588
#define STABILIZATION_MFC_ROLL_PROPORTIONAL_GAIN 4.
#define STABILIZATION_MFC_ROLL_COMMAND_FILTER 1.
#define STABILIZATION_MFC_PITCH_TIME_TRAJECTORY 50.
#define STABILIZATION_MFC_PITCH_INTEGRATION_WINDOW 5.
#define STABILIZATION_MFC_PITCH_ALPHA 147.0588
#define STABILIZATION_MFC_PITCH_PROPORTIONAL_GAIN 4.
#define STABILIZATION_MFC_PITCH_COMMAND_FILTER 1.
#define STABILIZATION_MFC_YAW_TIME_TRAJECTORY 50.
#define STABILIZATION_MFC_YAW_INTEGRATION_WINDOW 5.
#define STABILIZATION_MFC_YAW_ALPHA 73.5294
#define STABILIZATION_MFC_YAW_PROPORTIONAL_GAIN 4.
#define STABILIZATION_MFC_YAW_COMMAND_FILTER 1.

#define SECTION_STABILIZATION_ATTITUDE_INDI 1
#define STABILIZATION_INDI_REF_ERR_P 101
#define STABILIZATION_INDI_REF_ERR_Q 101
#define STABILIZATION_INDI_REF_ERR_R 124
#define STABILIZATION_INDI_REF_RATE_P 12.6
#define STABILIZATION_INDI_REF_RATE_Q 14.0
#define STABILIZATION_INDI_REF_RATE_R 14.0
#define STABILIZATION_INDI_MAX_R 1.04719755
#define STABILIZATION_INDI_FILT_CUTOFF 4.0
#define STABILIZATION_INDI_FILT_CUTOFF_R 4.0
#define STABILIZATION_INDI_USE_ADAPTIVE FALSE
#define STABILIZATION_INDI_ADAPTIVE_MU 0.0001
#define STABILIZATION_INDI_G1 { {-40 , -40, 40 , 40 } , {40 , -40, -40 , 40 } , {5, -5, 5, -5} , {-1.5, -1.5, -1.5, -1.5} }
#define STABILIZATION_INDI_G2 {150.0,   -150.0,  150.0,   -150.0 }
#define STABILIZATION_INDI_ACT_FREQ {30.5, 30.5, 30.5, 30.5}
#define STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF 4.0
#define STABILIZATION_INDI_WLS_PRIORITIES {1000, 1000, 1, 100}
#define STABILIZATION_INDI_COMMANDS {COMMAND_FR, COMMAND_BR, COMMAND_BL, COMMAND_FL}

#define SECTION_GUIDANCE_INDI 1
#define GUIDANCE_INDI_THRUST_DYNAMICS_FREQ 30.5
#define GUIDANCE_INDI_RC_DEBUG FALSE

#define SECTION_GUIDANCE_V 1
#define GUIDANCE_V_REF_MIN_ZDD -0.4*9.81
#define GUIDANCE_V_REF_MAX_ZDD  0.4*9.81
#define GUIDANCE_V_REF_MIN_ZD -1.5
#define GUIDANCE_V_REF_MAX_ZD  1.
#define GUIDANCE_V_NOMINAL_HOVER_THROTTLE 0.30

#define SECTION_GUIDANCE_H 1
#define GUIDANCE_H_MAX_BANK 0.34906585
#define GUIDANCE_H_REF_MAX_SPEED 2.5
#define GUIDANCE_H_REF_MAX_ACCEL 2.5

#define SECTION_GUIDANCE_MFC 1
#define GUIDANCE_MFC_GX_ALPHA 5.
#define GUIDANCE_MFC_GX_KP 1.
#define GUIDANCE_MFC_GX_TIME_TRAJECTORY 2.
#define GUIDANCE_MFC_GX_INTEGRATION_WINDOW 1.
#define GUIDANCE_MFC_GX_COMMAND_FILTER 1.
#define GUIDANCE_MFC_GY_ALPHA 5.
#define GUIDANCE_MFC_GY_KP 1.
#define GUIDANCE_MFC_GY_TIME_TRAJECTORY 2.
#define GUIDANCE_MFC_GY_INTEGRATION_WINDOW 1.
#define GUIDANCE_MFC_GY_COMMAND_FILTER 1.
#define GUIDANCE_MFC_GZ_ALPHA 32.7
#define GUIDANCE_MFC_GZ_KP 0.5
#define GUIDANCE_MFC_GZ_TIME_TRAJECTORY 2.
#define GUIDANCE_MFC_GZ_INTEGRATION_WINDOW 1.
#define GUIDANCE_MFC_GZ_COMMAND_FILTER 1.
#define GUIDANCE_MFC_GZ_NOMINAL_HOVER_THROTTLE 0.30

#define SECTION_NAV 1
#define ARRIVED_AT_WAYPOINT 2.0
#define NAV_CLIMB_VSPEED 1.5
#define NAV_DESCEND_VSPEED -0.8
#define RECTANGLE_SURVEY_HEADING_NS 0.

#define SECTION_BAT 1
#define MilliAmpereOfAdc(_adc) (20.76*_adc-9970)
#define CATASTROPHIC_BAT_LEVEL 12.3
#define CRITIC_BAT_LEVEL 13.
#define LOW_BAT_LEVEL 13.5
#define MAX_BAT_LEVEL 17.0
#define BAT_NB_CELLS 4

#define SECTION_AUTOPILOT 1
#define MODE_STARTUP AP_MODE_NAV
#define MODE_MANUAL AP_MODE_ATTITUDE_DIRECT
#define MODE_AUTO1 AP_MODE_ATTITUDE_DIRECT
#define MODE_AUTO2 AP_MODE_NAV

#define SECTION_SONAR 1
#define SONAR_ADC_SCALE 0.0025

#define SECTION_AGL 1
#define AGL_DIST_SONAR_ID ABI_BROADCAST
#define AGL_DIST_SONAR_MAX_RANGE 6.
#define AGL_DIST_SONAR_MIN_RANGE 0.01
#define AGL_DIST_SONAR_FILTER 0.15

#define SECTION_TAG_TRACKING 1
#define TAG_TRACKING_BODY_TO_CAM_PSI 0
#define TAG_TRACKING_CAM_POS_Y -0.12

#define SECTION_MISC 1
#define POWER_SWITCH_GPIO GPIOA,GPIO6

#define SECTION_GCS 1
#define ALT_SHIFT_PLUS_PLUS 3
#define ALT_SHIFT_PLUS 1
#define ALT_SHIFT_MINUS -0.5

#define SECTION_SIMULATOR 1
#define NPS_ACTUATOR_NAMES { "ne_motor" , "se_motor" , "sw_motor" , "nw_motor" }
#define NPS_COMMANDS_NB 4
#define NPS_JSBSIM_MODEL "anton"
#define NPS_NO_MOTOR_MIXING TRUE


#endif // AIRFRAME_H
