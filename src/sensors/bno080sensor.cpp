/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "sensors/bno080sensor.h"
#include "network/network.h"
#include "utils.h"
#include "GlobalVars.h"

#if BNO_USE_MADGWICK
    #include "madgwick.h"

    #define MADGWICK_BETA_STABLE_DETAULT_MAX 0.2f
    #define MADGWICK_BETA_STABLE_DETAULT_MIN 0.01f // Compensate for minimal gyro drift

    
    #define MADGWICK_UPDATE_RATE_MS 5.f

    #define USE_SMART true
    #define SMART_ANGLE_DIFF 12.5f / 2.f
    #define SMART_ACCEL_STABLE 0.8f
    #define SMART_CORRECT_TIME_MS 2000.f
    #define SMART_BETA 0.2f
#endif

#define CALIB_MODE_INIT 0
#define CALIB_MODE_WAIT_GRAVITY 1
#define CALIB_MODE_DONE 2

void BNO080Sensor::motionSetup()
{
#ifdef DEBUG_SENSOR
    imu.enableDebugging(Serial);
#endif
    if(!imu.begin(addr, Wire, m_IntPin)) {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to %s on 0x%02x. "
                  "Info: SW Version Major: 0x%02x "
                  "SW Version Minor: 0x%02x "
                  "SW Part Number: 0x%02x "
                  "SW Build Number: 0x%02x "
                  "SW Version Patch: 0x%02x", 
                  getIMUNameByType(sensorType), 
                  addr, 
                  imu.swMajor, 
                  imu.swMinor, 
                  imu.swPartNumber, 
                  imu.swBuildNumber, 
                  imu.swVersionPatch
                );

    imu.enableLinearAccelerometer(10);

#if BNO_USE_MADGWICK
    #if USE_6_AXIS
        imu.enableGyro(10);
        imu.enableAccelerometer(10);
    #else
        imu.enableGyro(10);
        imu.enableAccelerometer(10);
        imu.enableMagnetometer(10);
    #endif
#else
    #if USE_6_AXIS
        #if (IMU == IMU_BNO085 || IMU == IMU_BNO086) && BNO_USE_ARVR_STABILIZATION
        imu.enableARVRStabilizedGameRotationVector(10);
        #else
        imu.enableGameRotationVector(10);
        #endif

        #if BNO_USE_MAGNETOMETER_CORRECTION
        imu.enableRotationVector(1000);
        #endif
    #else
        #if (IMU == IMU_BNO085 || IMU == IMU_BNO086) && BNO_USE_ARVR_STABILIZATION
        imu.enableARVRStabilizedRotationVector(10);
        #else
        imu.enableRotationVector(10);
        #endif
    #endif
#endif


    imu.enableTapDetector(100);
    imu.enableStabilityClassifier(100);

    //For calibration flip detection
    imu.enableAccelerometer(10);

#if ENABLE_INSPECTION
    imu.enableRawGyro(10);
    imu.enableRawAccelerometer(10);
    imu.enableRawMagnetometer(10);
#endif

    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;
    calibrationMode = CALIB_MODE_INIT;
    lastcalibrationMode = millis();
    lastMadgwick = millis();
    
}

void BNO080Sensor::motionLoop()
{
    //Look for reports from the IMU
    while (imu.dataAvailable())
    {
        hadData = true;

#if ENABLE_INSPECTION
        {
            int16_t rX = imu.getRawGyroX();
            int16_t rY = imu.getRawGyroY();
            int16_t rZ = imu.getRawGyroZ();
            uint8_t rA = imu.getGyroAccuracy();

            int16_t aX = imu.getRawAccelX();
            int16_t aY = imu.getRawAccelY();
            int16_t aZ = imu.getRawAccelZ();
            uint8_t aA = imu.getAccelAccuracy();

            int16_t mX = imu.getRawMagX();
            int16_t mY = imu.getRawMagY();
            int16_t mZ = imu.getRawMagZ();
            uint8_t mA = imu.getMagAccuracy();

            Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, rA, aX, aY, aZ, aA, mX, mY, mZ, mA);
        }
#endif

        lastReset = 0;
        lastData = millis();

        switch(calibrationMode) 
        {
            case CALIB_MODE_INIT: 
            {
                calibrationMode = CALIB_MODE_DONE;

                float g_az = (imu.getAccelZ() / EARTH_GRAVITY);
                if(g_az < -0.75f) {
                    m_Logger.info("Flip front to confirm start calibration");

                    calibrationMode = CALIB_MODE_WAIT_GRAVITY;
                    lastcalibrationMode = millis();
                }
                break;
            }
            case CALIB_MODE_WAIT_GRAVITY:
            {
                ledManager.on();

                if(lastcalibrationMode + 5000 < millis())
                {
                    ledManager.off();
                    calibrationMode = CALIB_MODE_DONE;

                    float g_az = (imu.getAccelZ() / EARTH_GRAVITY);
                    if(g_az > 0.75f) {
                        m_Logger.debug("Starting calibration...");

                        startCalibration(0);
                    }
                }
                break;
            }
        }

        if(calibrationMode != CALIB_MODE_DONE)
            return;

#if BNO_USE_MADGWICK
        if (imu.hasNewGyro())
        {
    #if USE_6_AXIS
            // $HACK For some reason the bias is too low and gyro lacks behind a bit (adafruit).
            // Probably has something to do with the execution speed.
            float multi_bias = 1.0275f;

            uint8_t a;
            imu.getGyro(Gxyz[0], Gxyz[1], Gxyz[2], a);
            imu.getAccel(Axyz[0], Axyz[1], Axyz[2], a);
    #else
            // $HACK For some reason the bias is too low and gyro lacks behind a bit (adafruit).
            // Probably has something to do with the execution speed.
            float multi_bias = 1.1f;
            
            uint8_t a;
            imu.getGyro(Gxyz[0], Gxyz[1], Gxyz[2], a);
            imu.getAccel(Axyz[0], Axyz[1], Axyz[2], a);
            imu.getMag(Mxyz[0], Mxyz[1], Mxyz[2], a);
    #endif

            avgGxyz[0] += Gxyz[0] * multi_bias;
            avgGxyz[1] += Gxyz[1] * multi_bias;
            avgGxyz[2] += Gxyz[2] * multi_bias;
            avgGyroSamples++;
        }

        if ((millis() - lastMadgwick) > MADGWICK_UPDATE_RATE_MS && avgGyroSamples > 0)
        {
            float newGxyz[3];
            newGxyz[0] = avgGxyz[0] / avgGyroSamples;
            newGxyz[1] = avgGxyz[1] / avgGyroSamples;
            newGxyz[2] = avgGxyz[2] / avgGyroSamples;

            avgGxyz[0] = 0.f;
            avgGxyz[1] = 0.f;
            avgGxyz[2] = 0.f;
            avgGyroSamples = 0;

            doMadgwickUpdate(Axyz, newGxyz, Mxyz);

            if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
            {
                newData = true;
                lastQuatSent = quaternion;
            }
        }

#else // BNO_USE_MADGWICK
#if USE_6_AXIS
    if (imu.hasNewGameQuat())
    {
        imu.getGameQuat(quaternion.x, quaternion.y, quaternion.z, quaternion.w, calibrationAccuracy);
        quaternion *= sensorOffset;
    #if SEND_ACCELERATION
            {
                uint8_t acc;
                this->imu.getLinAccel(this->acceleration[0], this->acceleration[1], this->acceleration[2], acc);
            }
    #endif // SEND_ACCELERATION

    #if ENABLE_INSPECTION
            {
                Network::sendInspectionFusedIMUData(sensorId, quaternion);
            }
    #endif // ENABLE_INSPECTION

        if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
        {
            newData = true;
            lastQuatSent = quaternion;
        }
    }

    #if BNO_USE_MAGNETOMETER_CORRECTION
        if (imu.hasNewMagQuat())
        {
            imu.getMagQuat(magQuaternion.x, magQuaternion.y, magQuaternion.z, magQuaternion.w, magneticAccuracyEstimate, magCalibrationAccuracy);
            magQuaternion *= sensorOffset;

        #if ENABLE_INSPECTION
            {
                Network::sendInspectionCorrectionData(sensorId, quaternion);
            }
        #endif // ENABLE_INSPECTION

            newMagData = true;
        }
    #endif // BNO_USE_MAGNETOMETER_CORRECTION
#else // USE_6_AXIS

    if (imu.hasNewQuat())
    {
        imu.getQuat(quaternion.x, quaternion.y, quaternion.z, quaternion.w, magneticAccuracyEstimate, calibrationAccuracy);
        quaternion *= sensorOffset;

#if ENABLE_INSPECTION
        {
            Network::sendInspectionFusedIMUData(sensorId, quaternion);
        }
#endif // ENABLE_INSPECTION

        if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
        {
            newData = true;
            lastQuatSent = quaternion;
        }
    }
#endif // USE_6_AXIS

#endif // BNO_USE_MADGWICK
        if (imu.getTapDetected())
        {
            tap = imu.getTapDetector();
        }
        if (m_IntPin == 255 || imu.I2CTimedOut())
            break;
    }
    if (lastData + 1000 < millis() && configured)
    {
        while(true) {
            BNO080Error error = imu.readError();
            if(error.error_source == 255)
                break;
            lastError = error;
            m_Logger.error("BNO08X error. Severity: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d",
                error.severity, error.error_sequence_number, error.error_source, error.error, error.error_module, error.error_code);
        }
        statusManager.setStatus(SlimeVR::Status::IMU_ERROR, true);
        working = false;
        lastData = millis();
        uint8_t rr = imu.resetReason();
        if (rr != lastReset)
        {
            lastReset = rr;
            Network::sendError(rr, this->sensorId);
        }
        m_Logger.error("Sensor %d doesn't respond. Last reset reason:", sensorId, lastReset);
        m_Logger.error("Last error: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d",
                lastError.severity, lastError.error_sequence_number, lastError.error_source, lastError.error, lastError.error_module, lastError.error_code);
    }
}

uint8_t BNO080Sensor::getSensorState() {
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void BNO080Sensor::sendData()
{
    if (newData)
    {
        newData = false;
        Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, calibrationAccuracy, sensorId);

#if SEND_ACCELERATION
        Network::sendAccel(this->acceleration, this->sensorId);
#endif

#if !USE_6_AXIS
        Network::sendMagnetometerAccuracy(magneticAccuracyEstimate, sensorId);
#endif

#ifdef DEBUG_SENSOR
        m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(quaternion));
#endif
    }

#if USE_6_AXIS && BNO_USE_MAGNETOMETER_CORRECTION
    if (newMagData)
    {
        newMagData = false;
        Network::sendRotationData(&magQuaternion, DATA_TYPE_CORRECTION, magCalibrationAccuracy, sensorId);
        Network::sendMagnetometerAccuracy(magneticAccuracyEstimate, sensorId);
    }
#endif

    if (tap != 0)
    {
        Network::sendTap(tap, sensorId);
        tap = 0;
    }
}

void BNO080Sensor::startCalibration(int calibrationType)
{
    ledManager.pattern(15, 300, 3000/310);

    // Start calibration when device is not in motion
    // 0 - Unknown
    // 1 - On table
    // 2 - Stationary
    // 3 - Stable
    // 4 - Motion
    // 5 - Reserved
    do
    {
        ledManager.on();
        delay(20);
        imu.getReadings();
        ledManager.off();
        delay(20);
    } while (imu.getStabilityClassifier() != 1);
    
    // Start calibration
#if USE_6_AXIS
    imu.calibrateGyro();
#else
    imu.calibrateAll();
#endif

    // Wait for quick gyro calibration before saving
    ledManager.blink(10000);

    saveCalibration();

    ledManager.pattern(15, 300, 3000/310);

    // Wait for magnetometer/accelerometer calibration
    if(BNO_SELF_CALIBRATION_TIME > 0)
    {
        ledManager.blink(BNO_SELF_CALIBRATION_TIME);

        saveCalibration();

        imu.endCalibration();
    }
}

void BNO080Sensor::saveCalibration()
{
    do
    {
        ledManager.on();
        imu.requestCalibrationStatus();
        delay(20);
        imu.getReadings();
        ledManager.off();
        delay(20);
    } while (!imu.calibrationComplete());
    imu.saveCalibration();
}

#if BNO_USE_MADGWICK
void BNO080Sensor::doMadgwickUpdate(float Axyz[3], float Gxyz[3], float Mxyz[3]) {
    float lastDelta = (millis() - lastMadgwick) / 1000.f;

    //m_Logger.debug("lastDelta : %f", lastDelta);

#if USE_6_AXIS
    madgwickQuaternionUpdateStable(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], MADGWICK_BETA_STABLE_DETAULT_MIN, MADGWICK_BETA_STABLE_DETAULT_MAX, lastDelta);
#else
    madgwickQuaternionUpdateStable(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], MADGWICK_BETA_STABLE_DETAULT_MIN, MADGWICK_BETA_STABLE_DETAULT_MAX, lastDelta); //main
    madgwickQuaternionUpdate(s_q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], lastDelta, SMART_BETA); //smart
#endif
    lastMadgwick = millis();

#if !USE_6_AXIS && USE_SMART
    Quat quat;
    Quat quatSmart;
    quat.set(-q[2], q[1], q[3], q[0]);
    quatSmart.set(-s_q[2], s_q[1], s_q[3], s_q[0]);

    // Get angle in degrees
    const float angleDiff = fabsf(quatSmart.angle_to(quat) * (180.f / Math_PI));
    const float accel_q = sqrtf(Axyz[0] * Axyz[0] + Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]) / EARTH_GRAVITY;
    const float mag_q = sqrtf(Mxyz[0] * Mxyz[0] + Mxyz[1] * Mxyz[1] + Mxyz[2] * Mxyz[2]);
    const float gyro_q = (sqrtf(Gxyz[0] * Gxyz[0] + Gxyz[1] * Gxyz[1] + Gxyz[2] * Gxyz[2]) * lastDelta) * (180.f / PI);

    //m_Logger.debug("angleDiff : %f | accel_q : %f  | mag_q : %f| gyro_q : %f | lastSmart : %i ", angleDiff, accel_q, mag_q, gyro_q, lastSmart);

    // Smart reset when angle different is too big and accelerator is stable
    if ((angleDiff > SMART_ANGLE_DIFF) && (accel_q > SMART_ACCEL_STABLE && accel_q < 1.f + (1.f - SMART_ACCEL_STABLE))) {
        if (lastSmart == 0) {
            lastSmart = millis();
        }
        
        if ((millis() - lastSmart) > SMART_CORRECT_TIME_MS) {
            // Set smart quat to main quat
            q[0] = s_q[0];
            q[1] = s_q[1];
            q[2] = s_q[2];

            lastSmart = 0;
        }
    }
    else {
        lastSmart = 0;
    } 
#endif

    quaternion.set(-q[2], q[1], q[3], q[0]);
}
#endif