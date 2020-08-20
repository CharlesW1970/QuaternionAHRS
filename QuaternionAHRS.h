/**
  ******************************************************************************
  * @file    QuaternionAHRS.c
  * @author  Hongxi Wong
  * @version V1.0.2
  * @date    2020/8/20
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef QUAT_AHRS_H
#define QUAT_AHRS_H
#define ABS(x) ((x > 0) ? x : -x)

#include "stdint.h"

// algorithm parameter ---------------------------------------------------------------
#define SampleFreq 1000.0f / INS_TASK_PERIOD // sample frequency in Hz
#define twoKpDef (2.0f * 0.9f)               // 2 * proportional gain
#define twoKiDef (2.0f * 0.005f)             // 2 * integral gain
#define KMag 0.1f                            // magnetometer weight coefficient
#define MinACC 0.5f                          // MinACC times gravity
#define MaxACC 5.0f                          // MaxACC times gravity
#define DEADBAND 0.2f                        // deadband of weight self-adaption
//------------------------------------------------------------------------------------

typedef struct
{
    float q[4];

    float Roll;
    float Pitch;
    float Yaw;

    float Roll_Total_Angle;
    float Pitch_Total_Angle;
    float Yaw_Total_Angle;

    //int16_t Roll_Round_Count;
    int16_t Pitch_Round_Count;
    int16_t Yaw_Round_Count;
    //float Roll_Angle_Last;
    float Pitch_Angle_Last;
    float Yaw_Angle_Last;
} AHRS_t;

extern volatile float twoKp;          // 2 * proportional gain (Kp)
extern volatile float twoKi;          // 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
extern AHRS_t AHRS;

void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Quaternion_AHRS_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Get_EulerAngle(void);
float invSqrt(float x);

#endif
