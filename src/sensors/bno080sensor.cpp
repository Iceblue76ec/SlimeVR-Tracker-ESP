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

// 为什么定义在sensors/bno080sensor.h中会找不到代码实现，头大，暂时放这
void printAccuracyLevel(const char* text,uint8_t accuracyNumber);
void printStabilityClassifier(uint8_t StabilityNumber);

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

#if BNO_BOOT_AUTO_CALIBRATION
	// 状态判断器
	imu.enableStabilityClassifier(10);
	// 开启信息回报才能知道精度参数
	imu.enableAccelerometer(50);
	imu.enableGyro(50);
	imu.enableMagnetometer(50);
#endif
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

		#if BNO_BOOT_AUTO_CALIBRATION
		// 开机校准相关
		if (!calibStopped && millis()%100==0) {
			// 串口打印运动状态
			printStabilityClassifier(imu.getStabilityClassifier());
            printAccuracyLevel("accelAccuracy",imu.getAccelAccuracy());
            printAccuracyLevel("gyroAccuracy",imu.getGyroAccuracy());
            //m_Logger.info("当设备开机15秒内 且 在桌面上 且陀螺仪精度低于高 启动校准");
			if (!calibStarted && millis() < 15000 && imu.getStabilityClassifier() == 1 && imu.getGyroAccuracy()<3) {
				// 启动校准
                m_Logger.info("进入校准");
                imu.calibrateGyro();
                m_Logger.info("陀螺仪校准");
				calibStarted = true;
				// 校准时LED快速闪烁
				ledManager.blink(150);  
			}
			// 校准已经启动，且校准未完成
			if (calibStarted && !calibStopped) {
				if (millis() % 200 == 0) {
					m_Logger.info("校准中……");
					// 串口打印精度
					{
						printAccuracyLevel("accelAccuracy",imu.getAccelAccuracy());
						printAccuracyLevel("gyroAccuracy",imu.getGyroAccuracy());
                        //#if calibrateAll
						//printAccuracyLevel("magAccuracy",imu.getMagAccuracy());
                        //#endif
					}
				}
                // 单独陀螺仪保存校准，加速度计校准状态 且 陀螺仪校准精度低于high 不保存
                if (!calibAccelerometerStarted && imu.getGyroAccuracy() == 3) 
                {
					imu.saveCalibration();
					imu.endCalibration();
					m_Logger.info("陀螺仪精度合格，保存校准");
                    //陀螺仪校准参数保存之后再开始加速度计校准
                    imu.calibrateAccelerometer();
                    m_Logger.info("加速度计校准");
				    calibAccelerometerStarted = true;
                    // 校准时LED快速闪烁
				    ledManager.blink(150);
                }
				// 单独加速度计保存校准，加速度计校准精度低于high 不保存
				if (imu.getAccelAccuracy() == 3) {
					imu.saveCalibration();
					imu.endCalibration();
					calibStopped = true;
					m_Logger.info("加速度计精度合格，保存校准");
				}
			}
			// 如果预设校准时间结束(开机30秒内) 或
			// slime处于运动状态，则校准强制结束且不保存校准信息 
			if (millis() > 30000 && !calibStopped) {
				imu.endCalibration();
				calibStopped = true;
				ledManager.off();
				m_Logger.info("自动校准结束");
                #if BNO_USE_MAGNETOMETER_CORRECTION
				// 9轴模式一直保持磁力计自校准也许对环境适应能力更强
					imu.calibrateMagnetometer();
                    m_Logger.info("Mag修正模式,保留Mag校准");
                    printAccuracyLevel("magAccuracy",imu.getMagAccuracy());
				#endif
                printAccuracyLevel("accelAccuracy",imu.getAccelAccuracy());
				printAccuracyLevel("gyroAccuracy",imu.getGyroAccuracy());
			}
		}
		#endif //选择性编译开机自校准部分
	}
	// 异常状态处理
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
void printAccuracyLevel(const char* text,uint8_t accuracyNumber) {

	Serial.print("[INFO ] [BNO080Sensor] ");
	Serial.print(text);
	Serial.print(" : ");
	switch (accuracyNumber)
	{
	case 0:Serial.print("Unreliable");break;
	case 1:Serial.print("Low");break;
	case 2:Serial.print("Medium");break;
	case 3:Serial.print("High");break;
	default:
		break;
	}
	Serial.println();
}

void printStabilityClassifier(uint8_t stabilityNumber)
{
	// 0 - 未知
    // 1 - 在桌子上
    // 2 - 静止
    // 3 - 稳定
    // 4 - 运动
    // 5 - 保留
	char text[40]="[INFO ] [BNO080Sensor] ";
	switch (stabilityNumber)
	{
	case 1:Serial.println(strcat(text,"状态:桌上"));break;
	case 2:Serial.println(strcat(text,"状态:稳定"));break;
	case 3:Serial.println(strcat(text,"状态:静止"));break;
	case 4:Serial.println(strcat(text,"状态:运动"));break;
	default:Serial.println(strcat(text,"状态:未知"));break;
	}
}

