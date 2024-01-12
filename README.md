# 关于BNO085自动校准固件
该文档记录了关于BNO08X开机自校准的尝试，预期是让BNO能达到最理想的精度和稳定性。
## 1.第一个自动校准校准版本

从github仓库：https://github.com/Timocop/PSMoveServiceEx-SlimeVR-Tracker-ESP

的历史提交中考古出来

### BNO080Sensor::motionSetup(){

```c++
void BNO080Sensor::motionSetup(){
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

    imu.enableTapDetector(100);
    //判断085的运动状态 在桌上/静止/运动
    imu.enableStabilityClassifier(100);

#if ENABLE_INSPECTION
    imu.enableRawGyro(10);
    imu.enableRawAccelerometer(10);
    imu.enableRawMagnetometer(10);
#endif

    lastReset = 0;
    lastData = millis();
    working = true;
    configured = true;
    lastCalib = millis();
    calibStopped = false;

    // startCalibration() executes too soon before parameters have set
    initCalibration();
}
```

### src/debug.h

配置里多了一个参数

```c++
src/debug.h
#define BNO_USE_MAGNETOMETER_CORRECTION false // Set to true to enable magnetometer correction for BNO08x IMUs. Only works with USE_6_AXIS set to true
#define BNO_SELF_CALIBRATION_TIME 30000 // 如果 BNO8x 应及时停止自校准，则将值设置为非零（以毫秒为单位）
#define USE_6_AXIS true // uses 9 DoF (with mag) if false (only for ICM-20948 and BNO0xx currently)
```

### BNO080Sensor::motionLoop(){

运行时校准新：

```c++
void BNO080Sensor::motionLoop(){
    // $TODO 只保存好的准确度
    if(BNO_SELF_CALIBRATION_TIME > 0 && lastCalib + BNO_SELF_CALIBRATION_TIME < millis() && !calibStopped)
    {
        calibStopped = true;
        saveCalibration();
        imu.endCalibration();
        m_Logger.info("Calibration ended");
        m_Logger.info("Calibration accuracy: %d", calibrationAccuracy);
    }
}
```

运行时校准旧：

缺点，会暂停整个主线程

```c++
void BNO080Sensor::motionLoop(){
	if(BNO_SELF_CALIBRATION_TIME > 0 && lastCalib + BNO_SELF_CALIBRATION_TIME < millis() && !calibStopped)
    {
        calibStopped = true;

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

        imu.endCalibration();
        m_Logger.error("Calibration ended");
    }
	}
}
```

### BNO080Sensor::initCalibration()

开机时仅在稳定时才开始校准

```c++
void BNO080Sensor::initCalibration()
{
    ledManager.pattern(20, 20, 10);
    ledManager.blink(2000);

    //当设备不运动时开始校准
    // 0 - 未知
    // 1 - 在桌子上
    // 2 - 静止
    // 3 - 稳定
    // 4 - 运动
    // 5 - 保留

    do
    {
        ledManager.on();
        delay(20);
        imu.getReadings();
        ledManager.off();
        delay(20);

        if(imu.getStabilityClassifier() == 4)
        {
            calibStopped = true;
            return;
        }
    } while (imu.getStabilityClassifier() != 1);

    // Start calibration
#if USE_6_AXIS
    imu.calibrateGyro();
#else
    imu.calibrateAll();
#endif

    // 保存前等待快速陀螺仪校准
    ledManager.blink(10000);

    saveCalibration();

    lastCalib = millis();
}
```

### BNO080Sensor::saveCalibration()

保存校准

```c++
void BNO080Sensor::saveCalibration()
{
    do
    {
        ledManager.on();
         delay(20);
    } while (!imu.calibrationComplete());
    imu.saveCalibration();
}
```

### src/sensors/bno080sensor.h

声明这两个函数，和校准变量

```c++
src/sensors/bno080sensor.h
class BNO080Sensor : public Sensor

void startCalibration(int calibrationType) override final;
    uint8_t getSensorState() override final;

    void initCalibration();
    void saveCalibration();

	uint8_t lastCalib = 0;
    bool calibStopped = false;

private:
    BNO080 imu{};

```

## 2.第二个自动校准版本

逻辑：开机时检测085是否在桌面，确定在桌面后，校准陀螺仪，陀螺仪精度合格则自动保存（校准过程不到1秒，太仓促了）。

## 3.第三个自动校准版本

逻辑：开机2.5秒后，记录开机时陀螺仪、加速度计精度，开启自动校准。30秒后，当前精度比开机时精度高则保存。校准时间更长，可采集数据更多，理论上校准更可靠。

