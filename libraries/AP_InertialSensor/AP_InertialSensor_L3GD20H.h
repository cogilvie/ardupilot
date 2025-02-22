#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_HAL/I2CDevice.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_L3GD20H : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_L3GD20H(AP_InertialSensor &imu,
                               AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    virtual ~AP_InertialSensor_L3GD20H();

    // probe the sensor on I2C bus
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /* update gyro state */
    bool update() override;

    void start(void) override;

private:
    bool _init_sensor();
    void _accumulate();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    // gyro instance
    uint8_t _gyro_instance;
};
#endif
