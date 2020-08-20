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
#include "QuaternionAHRS.h"
#include <math.h>

AHRS_t AHRS = {0};

volatile float twoKp = twoKpDef; // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef; // 2 * integral gain (Ki)
volatile float AccWeight = 1;
volatile float MagWeight = KMag; // magnetometer weight coefficient

static float ACC_Norm = 0;
const float Weightlessness = 9.8f * MinACC;
const float Superheavy = 9.8f * MaxACC;
static float Coef1 = 9.8f - Weightlessness;
static float Coef2 = Superheavy - 9.8f;

volatile float sampleFreq = SampleFreq;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

static float invSqrt(float x);

/**
  * @brief          AHRS algorithm update
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s2
  * @param[7-9]     magnetometer measurement in uT
  */
void Quaternion_AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Quaternion_AHRS_UpdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    ACC_Norm = sqrt(ax * ax + ay * ay + az * az);

    if (ACC_Norm >= Weightlessness && ACC_Norm <= Superheavy)
    {
        if (ABS(ACC_Norm - 9.8f) >= DEADBAND)
        {
            if (ACC_Norm < 9.8f)
            {
                AccWeight = 1 - ((9.8f - ACC_Norm) / Coef1);
            }
            else
            {
                AccWeight = 1 - ((ACC_Norm - 9.8f) / Coef2);
            }
        }
        else
        {
            AccWeight = 1;
        }
    }
    else
    {
        ax = 0.0f;
        ay = 0.0f;
        az = 0.0f;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) * AccWeight + (my * halfwz - mz * halfwy) * MagWeight;
        halfey = (az * halfvx - ax * halfvz) * AccWeight + (mz * halfwx - mx * halfwz) * MagWeight;
        halfez = (ax * halfvy - ay * halfvx) * AccWeight + (mx * halfwy - my * halfwx) * MagWeight;

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    AHRS.q[0] = q0;
    AHRS.q[1] = q1;
    AHRS.q[2] = q2;
    AHRS.q[3] = q3;
}

/**
  * @brief          IMU algorithm update
  * @param[1-3]     gyro measurement in rad/s
  * @param[4-6]     accelerometer measurement in m/s2
  */
void Quaternion_AHRS_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    ACC_Norm = sqrt(ax * ax + ay * ay + az * az);

    if (ACC_Norm >= Weightlessness && ACC_Norm <= Superheavy)
    {
        if (ABS(ACC_Norm - 9.8f) >= DEADBAND)
        {
            if (ACC_Norm < 9.8f)
            {
                AccWeight = 1 - ((9.8f - ACC_Norm) / Coef1);
            }
            else
            {
                AccWeight = 1 - ((ACC_Norm - 9.8f) / Coef2);
            }
        }
        else
        {
            AccWeight = 1;
        }
    }
    else
    {
        twoKp = 0;
        ax = 0.0f;
        ay = 0.0f;
        az = 0.0f;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy) * AccWeight;
        halfey = (az * halfvx - ax * halfvz) * AccWeight;
        halfez = (ax * halfvy - ay * halfvx) * AccWeight;

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    AHRS.q[0] = q0;
    AHRS.q[1] = q1;
    AHRS.q[2] = q2;
    AHRS.q[3] = q3;
}

/**
  * @brief        Convert quaternion to eular angle
  */
void Get_EulerAngle(void)
{
    static uint8_t count = 0;

    AHRS.Yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 2.0f * (q0 * q0 + q1 * q1) - 1.0f) * 57.295779513f;
    AHRS.Pitch = atan2f(2.0f * (q0 * q1 + q2 * q3), 2.0f * (q0 * q0 + q3 * q3) - 1.0f) * 57.295779513f;
    AHRS.Roll = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 57.295779513f;

    if (count == 0)
    {
        AHRS.Yaw_Angle_Last = AHRS.Yaw;
        AHRS.Pitch_Angle_Last = AHRS.Pitch;
        AHRS.Roll_Angle_Last = AHRS.Roll;
    }

    // Yaw rount count
    if (AHRS.Yaw - AHRS.Yaw_Angle_Last > 180.0f)
        AHRS.Yaw_Round_Count--;
    else if (AHRS.Yaw - AHRS.Yaw_Angle_Last < -180.0f)
        AHRS.Yaw_Round_Count++;
    // Pitch rount count
    if (AHRS.Pitch - AHRS.Pitch_Angle_Last > 180.0f)
        AHRS.Pitch_Round_Count--;
    else if (AHRS.Pitch - AHRS.Pitch_Angle_Last < -180.0f)
        AHRS.Pitch_Round_Count++;
    // Roll rount count
    // if (AHRS.Roll - AHRS.Roll_Angle_Last > 180.0f)
    //     AHRS.Roll_Round_Count--;
    // else if (AHRS.Roll - AHRS.Roll_Angle_Last < -180.0f)
    //     AHRS.Roll_Round_Count++;

    AHRS.Yaw_Total_Angle = 360.0f * AHRS.Yaw_Round_Count + AHRS.Yaw;
    AHRS.Pitch_Total_Angle = 360.0f * AHRS.Pitch_Round_Count + AHRS.Pitch;
    // AHRS.Roll_Total_Angle = 360.0f * AHRS.Roll_Round_Count + AHRS.Roll;

    AHRS.Yaw_Angle_Last = AHRS.Yaw;
    AHRS.Pitch_Angle_Last = AHRS.Pitch;
    // AHRS.Roll_Angle_Last = AHRS.Roll;

    count = 1;
}

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
