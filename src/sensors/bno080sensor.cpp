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
#include "utils.h"
#include "GlobalVars.h"

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

    this->imu.enableLinearAccelerometer(10);

#if USE_6_AXIS
    if ((sensorType == IMU_BNO085 || sensorType == IMU_BNO086) && BNO_USE_ARVR_STABILIZATION) {
        imu.enableARVRStabilizedGameRotationVector(10);
		m_Logger.info("enable ARVR Stabilized Game Rotation Vector");
    } else {
        imu.enableGameRotationVector(10);
		m_Logger.info("enable Game Rotation Vector");
    }

    #if BNO_USE_MAGNETOMETER_CORRECTION
    imu.enableRotationVector(1000);
	m_Logger.info("enable Rotation Vector");
    #endif
#else
    if ((sensorType == IMU_BNO085 || sensorType == IMU_BNO086) && BNO_USE_ARVR_STABILIZATION) {
        imu.enableARVRStabilizedRotationVector(10);
		m_Logger.info("enable ARVR Stabilized Rotation Vector");
    } else {
        imu.enableRotationVector(10);
		m_Logger.info("enable Rotation Vector");
    }
#endif

#if ENABLE_INSPECTION
    imu.enableRawGyro(10);
    imu.enableRawAccelerometer(10);
    imu.enableRawMagnetometer(10);
#endif
    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;

	imu.enableStabilityClassifier(100);
	lastMotionSetupTime = millis();
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

            networkConnection.sendInspectionRawIMUData(sensorId, rX, rY, rZ, rA, aX, aY, aZ, aA, mX, mY, mZ, mA);
        }
#endif

        lastReset = 0;
        lastData = millis();

#if USE_6_AXIS
        if (imu.hasNewGameQuat()) // New quaternion if context
        {
            imu.getGameQuat(fusedRotation.x, fusedRotation.y, fusedRotation.z, fusedRotation.w, calibrationAccuracy);
            fusedRotation *= sensorOffset;

            setFusedRotationReady();
            // Leave new quaternion if context open, it's closed later

#else // USE_6_AXIS

        if (imu.hasNewQuat()) // New quaternion if context
        {
            imu.getQuat(fusedRotation.x, fusedRotation.y, fusedRotation.z, fusedRotation.w, magneticAccuracyEstimate, calibrationAccuracy);
            fusedRotation *= sensorOffset;

            setFusedRotationReady();
            // Leave new quaternion if context open, it's closed later
#endif // USE_6_AXIS

            // Continuation of the new quaternion if context, used for both 6 and 9 axis
#if SEND_ACCELERATION
            {
                uint8_t acc;
                imu.getLinAccel(acceleration.x, acceleration.y, acceleration.z, acc);
                setAccelerationReady();
            }
#endif // SEND_ACCELERATION
        } // Closing new quaternion if context

#if USE_6_AXIS && BNO_USE_MAGNETOMETER_CORRECTION
        if (imu.hasNewMagQuat())
        {
            imu.getMagQuat(magQuaternion.x, magQuaternion.y, magQuaternion.z, magQuaternion.w, magneticAccuracyEstimate, magCalibrationAccuracy);
            magQuaternion *= sensorOffset;

    #if ENABLE_INSPECTION
            {
                networkConnection.sendInspectionCorrectionData(sensorId, quaternion);
            }
    #endif // ENABLE_INSPECTION

            newMagData = true;
        }
#endif // USE_6_AXIS && BNO_USE_MAGNETOMETER_CORRECTION

        if (imu.getTapDetected())
        {
            tap = imu.getTapDetector();
        }
        if (m_IntPin == 255 || imu.I2CTimedOut())
            break;
    }
	// 异常状态
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
            networkConnection.sendSensorError(this->sensorId, rr);
        }

        m_Logger.error("Sensor %d doesn't respond. Last reset reason:", sensorId, lastReset);
        m_Logger.error("Last error: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d",
                lastError.severity, lastError.error_sequence_number, lastError.error_source, lastError.error, lastError.error_module, lastError.error_code);
    }

	// 开机校准相关
	if (!calibStopped) {
		// 串口调试信息测试
		if((millis()-lastMotionSetupTime)%1000==0){
			m_Logger.info("状态：0-未知 1-桌上 2-静止 3-稳定");
			printStabilityClassifier(imu.getStabilityClassifier());
			m_Logger.info("加速度计精度：");
			printAccuracyLevel(imu.getAccelAccuracy());
			m_Logger.info("陀螺仪计精度：");
			printAccuracyLevel(imu.getGyroAccuracy());
			m_Logger.info("磁力计计精度：");
			printAccuracyLevel(imu.getMagAccuracy());
			m_Logger.info("四元数精度：");
			printAccuracyLevel(imu.getQuatAccuracy());
		}

		// 当设备开机15秒内不运动时启动校准
		if (!calibStarted && lastMotionSetupTime + 15000 < millis() && imu.getStabilityClassifier() == 1) {
			//getStabilityClassifier需要10秒来判断状态
			// 0 - 未知
			// 1 - 在桌子上
			// 2 - 静止
			// 3 - 稳定
			// 4 - 运动
			// 5 - 保留

			// 启动校准
			#if BNO_USE_MAGNETOMETER_CORRECTION
			imu.calibrateAll();
			#else
			// todo：桌面平放状态没法考虑到加速度计的校准
			imu.calibrateGyro();
			#endif
			calibStarted=true;
			// 校准时LED快速闪烁
			ledManager.blink(150);
			m_Logger.info("桌面条件满足，启动校准");
		}
		// 校准已经启动，且校准未完成
		if (calibStarted && !calibStopped) {

			m_Logger.info("校准中……");
			//保存校准，陀螺仪校准精度低于high不保存
			if(imu.getGyroAccuracy()>=3){
				imu.saveCalibration();
				imu.endCalibration();
				calibStopped=true;
				#if BNO_USE_MAGNETOMETER_CORRECTION
				// 9轴模式一直保持磁力计自校准也许对环境适应能力更强
				imu.calibrateMagnetometer();
				#endif
				m_Logger.info("精度合格，保存校准");
			}
		}
		// 如果预设校准时间结束 或 slime未保持桌面静止状态，则校准强制结束且不保存校准信息
		if(lastMotionSetupTime + BNO_SELF_CALIBRATION_TIME > millis() || imu.getStabilityClassifier() != 1){
			imu.endCalibration();

			calibStarted=true;
			ledManager.off();
		}
	}
}

SensorStatus BNO080Sensor::getSensorState() {
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void BNO080Sensor::sendData()
{
    if (newFusedRotation)
    {
        newFusedRotation = false;
        networkConnection.sendRotationData(sensorId, &fusedRotation, DATA_TYPE_NORMAL, calibrationAccuracy);

#ifdef DEBUG_SENSOR
        m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(fusedRotation));
#endif

#if SEND_ACCELERATION
        if (newAcceleration)
        {
            newAcceleration = false;
            networkConnection.sendSensorAcceleration(this->sensorId, this->acceleration);
        }
#endif
    }

#if !USE_6_AXIS
        networkConnection.sendMagnetometerAccuracy(sensorId, magneticAccuracyEstimate);
#endif

#if USE_6_AXIS && BNO_USE_MAGNETOMETER_CORRECTION
    if (newMagData)
    {
        newMagData = false;
        networkConnection.sendRotationData(sensorId, &magQuaternion, DATA_TYPE_CORRECTION, magCalibrationAccuracy);
        networkConnection.sendMagnetometerAccuracy(sensorId, magneticAccuracyEstimate);
    }
#endif

    if (tap != 0)
    {
        networkConnection.sendSensorTap(sensorId, tap);
        tap = 0;
    }
}

void BNO080Sensor::startCalibration(int calibrationType)
{
    // BNO does automatic calibration,
    // it's always enabled except accelerometer
    // that is disabled 30 seconds after startup
}

// Given a accuracy number, print what it means
void printAccuracyLevel(uint8_t accuracyNumber)
{
  if (accuracyNumber == 0)
    Serial.print(F("Unreliable"));
  else if (accuracyNumber == 1)
    Serial.print(F("Low"));
  else if (accuracyNumber == 2)
    Serial.print(F("Medium"));
  else if (accuracyNumber == 3)
    Serial.print(F("High"));
}

void printStabilityClassifier(uint8_t StabilityClassifierNumber)
{
	if (StabilityClassifierNumber == 0)
    Serial.print(F("StabilityClassifier:未知"));
  else if (StabilityClassifierNumber == 1)
    Serial.print(F("StabilityClassifier:桌上"));
  else if (StabilityClassifierNumber == 2)
    Serial.print(F("StabilityClassifier:静止"));
  else if (StabilityClassifierNumber == 3)
    Serial.print(F("StabilityClassifier:稳定"));
}
