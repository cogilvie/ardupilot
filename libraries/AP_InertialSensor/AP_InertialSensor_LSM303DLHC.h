#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_LSM303DLHC : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_LSM303DLHC(AP_InertialSensor &imu,
                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    virtual ~AP_InertialSensor_LSM303DLHC();


    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                               AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

     /* update accel state */
     bool update() override;

     void start(void) override;

 private:
     uint8_t _register_read(uint8_t reg);
     void _register_write(uint8_t reg, uint8_t val);
     void _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);
     bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);

     bool _init_sensor();
     void _accumulate();

     AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

     // accel instance
     uint8_t _accel_instance;
};
