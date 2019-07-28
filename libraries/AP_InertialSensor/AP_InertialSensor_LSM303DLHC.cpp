/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_LSM303DLHC.h"

extern const AP_HAL::HAL &hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/GPIO.h>
#endif

#define DIR_READ                (1<<7)
#define DIR_WRITE               (0<<7)
#define ADDR_INCREMENT          (1<<6)

#define ADDR_WHO_AM_I           0x0F
#define WHO_I_AM                0x33

//  LSM303DLHC Accel Register Map

#define ADDR_CTRL1_A         0x20
#define ADDR_CTRL2_A         0x21
#define ADDR_CTRL3_A         0x22
#define ADDR_CTRL4_A         0x23
#define ADDR_CTRL5_A         0x24
#define ADDR_CTRL6_A         0x25
#define ADDR_REF_A           0x26
#define ADDR_STATUS_A        0x27
#define ADDR_OUT_X_L_A       0x28
#define ADDR_OUT_X_H_A       0x29
#define ADDR_OUT_Y_L_A       0x2a
#define ADDR_OUT_Y_H_A       0x2b
#define ADDR_OUT_Z_L_A       0x2c
#define ADDR_OUT_Z_H_A       0x2d
#define ADDR_FIFO_CTRL_A     0x2e
#define ADDR_FIFO_SRC_A      0x2f

//  LSM303DLHC Compass Register Map

#define ADDR_CRA_M            0x00
#define ADDR_CRB_M            0x01
#define ADDR_CRM_M            0x02
#define ADDR_OUT_X_H_M        0x03
#define ADDR_OUT_X_L_M        0x04
#define ADDR_OUT_Y_H_M        0x05
#define ADDR_OUT_Y_L_M        0x06
#define ADDR_OUT_Z_H_M        0x07
#define ADDR_OUT_Z_L_M        0x08
#define ADDR_STATUS_M         0x09
#define ADDR_TEMP_OUT_L_M     0x31
#define ADDR_TEMP_OUT_H_M     0x32


//  Accel FSR

#define ADDR_ACCEL_FSR_2     0
#define ADDR_ACCEL_FSR_4     1
#define ADDR_ACCEL_FSR_8     2
#define ADDR_ACCEL_FSR_16    3

//  Compass sample rate defines

#define ADDR_COMPASS_SAMPLERATE_0_75      0
#define ADDR_COMPASS_SAMPLERATE_1_5       1
#define ADDR_COMPASS_SAMPLERATE_3         2
#define ADDR_COMPASS_SAMPLERATE_7_5       3
#define ADDR_COMPASS_SAMPLERATE_15        4
#define ADDR_COMPASS_SAMPLERATE_30        5
#define ADDR_COMPASS_SAMPLERATE_75        6
#define ADDR_COMPASS_SAMPLERATE_220       7

#define CRA_ENABLE_T            (1<<7)

#define CRA_RATE_BITS_M         ((1<<4) | (1<<3) | (1<<2))
#define CRA_RATE_0_75HZ_M       ((0<<4) | (0<<3) | (0<<2))
#define CRA_RATE_1HZ_M          ((0<<4) | (0<<3) | (1<<2))
#define CRA_RATE_3HZ_M          ((0<<4) | (1<<3) | (0<<2))
#define CRA_RATE_7_5HZ_M        ((0<<4) | (1<<3) | (1<<2))
#define CRA_RATE_15HZ_M         ((1<<4) | (0<<3) | (0<<2))
#define CRA_RATE_30HZ_M         ((1<<4) | (0<<3) | (1<<2))
#define CRA_RATE_75HZ_M         ((1<<4) | (1<<3) | (0<<2))
#define CRA_RATE_220HZ_M        ((1<<4) | (1<<3) | (1<<2))


#define CRB_FULL_SCALE_BITS_M   ((1<<7) | (1<<6) | (1<<5))
#define CRB_FULL_SCALE_1_3GA_M  ((0<<7) | (0<<6) | (1<<5))
#define CRB_FULL_SCALE_1_9GA_M  ((0<<7) | (1<<6) | (0<<5))
#define CRB_FULL_SCALE_2_5GA_M  ((0<<7) | (1<<6) | (1<<5))
#define CRB_FULL_SCALE_4_0GA_M  ((1<<7) | (0<<6) | (0<<5))
#define CRB_FULL_SCALE_4_7GA_M  ((1<<7) | (0<<6) | (1<<5))
#define CRB_FULL_SCALE_5_6GA_M  ((1<<7) | (1<<6) | (0<<5))
#define CRB_FULL_SCALE_8_1GA_M  ((1<<7) | (1<<6) | (1<<5))

#define INT_CTRL_M              0x12
#define INT_SRC_M               0x13

#define LSM303DLHC_MAG_DEFAULT_RANGE_GA        1.9f
#define LSM303DLHC_MAG_DEFAULT_RATE            220

// constructor
AP_InertialSensor_LSM303DLHC::AP_InertialSensor_LSM303DLHC(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_LSM303DLHC::~AP_InertialSensor_LSM303DLHC()
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_LSM303DLHC::probe(AP_InertialSensor &imu,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_LSM303DLHC *sensor
        = new AP_InertialSensor_LSM303DLHC(imu, std::move(dev));
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

uint8_t AP_InertialSensor_LSM303DLHC::_register_read(uint8_t reg)
{
    uint8_t val = 0;

    reg |= DIR_READ;
    _dev->read_registers(reg, &val, 1);

    return val;
}

bool AP_InertialSensor_LSM303DLHC::_block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    reg |= DIR_READ;
    return _dev->read_registers(reg, buf, size);
}

void AP_InertialSensor_LSM303DLHC::_register_write(uint8_t reg, uint8_t val)
{
    _dev->write_register(reg, val);
}

void AP_InertialSensor_LSM303DLHC::_register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t val;

    val = _register_read(reg);
    val &= ~clearbits;
    val |= setbits;
    _register_write(reg, val);
}

bool AP_InertialSensor_LSM303DLHC::_init_sensor(void)
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
      AP_HAL::panic("LSM303DLHC: Unable to get semaphore");
    }

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // Test WHOAMI
    uint8_t whoami = _register_read(ADDR_WHO_AM_I);
    if (whoami != WHO_I_AM) {
        hal.console->printf("Accel LSM303DLHC: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
        goto fail_whoami;
    }

    //
    // // setup for 800Hz sampling with 110Hz filter
    // _dev->write_register(L3GD20H_REG_CTRL_REG1,
    //                      L3GD20H_REG_CTRL_REG1_DRBW_800_110 |
    //                      L3GD20H_REG_CTRL_REG1_PD |
    //                      L3GD20H_REG_CTRL_REG1_XYZ_ENABLE);
    // hal.scheduler->delay(1);
    //
    // // setup for 2000 degrees/sec full range
    // _dev->write_register(L3GD20H_REG_CTRL_REG4,
    //                      L3GD20H_REG_CTRL_REG4_FS_2000);
    // hal.scheduler->delay(1);
    //
    // // enable FIFO
    // _dev->write_register(L3GD20H_REG_CTRL_REG5,
    //                      L3GD20H_REG_CTRL_REG5_FIFO_EN);
    // hal.scheduler->delay(1);
    //
    // // enable FIFO in stream mode. This will allow us to read the gyros much less frequently
    // _dev->write_register(L3GD20H_REG_FIFO_CTL,
    //                      L3GD20H_REG_FIFO_CTL_STREAM);
    // hal.scheduler->delay(1);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();

    return true;

fail_whoami:
    _dev->get_semaphore()->give();
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    return false;
}

/*
  startup the sensor
 */
void AP_InertialSensor_LSM303DLHC::start(void)
{
    _accel_instance = _imu.register_accel(800, _dev->get_bus_id_devtype(DEVTYPE_ACC_LSM303DLHC));

    // start the timer process to read samples
    _dev->register_periodic_callback(1250, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM303DLHC::_accumulate, void));
}

/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_LSM303DLHC::update(void)
{
    update_accel(_accel_instance);

    return true;
}

// Accumulate values from accels
void AP_InertialSensor_LSM303DLHC::_accumulate(void)
{
    // uint8_t num_samples_available;
    // uint8_t fifo_status = 0;
    //
    // // Read gyro FIFO status
    // fifo_status = 0;
    // _dev->read_registers(L3GD20H_REG_FIFO_SRC, &fifo_status, 1);
    // if (fifo_status & L3GD20H_REG_FIFO_SRC_OVERRUN) {
    //     // FIFO is full
    //     num_samples_available = 32;
    // } else if (fifo_status & L3GD20H_REG_FIFO_SRC_EMPTY) {
    //     // FIFO is empty
    //     num_samples_available = 0;
    // } else {
    //     // FIFO is partly full
    //     num_samples_available = fifo_status & L3GD20H_REG_FIFO_SRC_ENTRIES_MASK;
    // }
    //
    // if (num_samples_available > 0) {
    //     // read all the entries in one go, using AUTO_INCREMENT. This saves a lot of time on I2C setup
    //     int16_t buffer[num_samples_available][3];
    //     if (!_dev->read_registers(L3GD20H_REG_XL | L3GD20H_REG_AUTO_INCREMENT,
    //                               (uint8_t *)&buffer, sizeof(buffer))) {
    //         for (uint8_t i=0; i < num_samples_available; i++) {
    //             Vector3f gyro = Vector3f(buffer[i][0], -buffer[i][1], -buffer[i][2]);
    //             // Adjust for chip scaling to get radians/sec
    //             gyro *= L3GD20H_GYRO_SCALE_R_S;
    //             _rotate_and_correct_gyro(_gyro_instance, gyro);
    //             _notify_new_gyro_raw_sample(_gyro_instance, gyro);
    //         }
    //     }
    // }
}







// uint8_t AP_Compass_LSM303DLHC::_register_read(uint8_t reg)
// {
//     uint8_t val = 0;
//
//     reg |= DIR_READ;
//     _dev->read_registers(reg, &val, 1);
//
//     return val;
// }
//
// bool AP_Compass_LSM303DLHC::_block_read(uint8_t reg, uint8_t *buf, uint32_t size)
// {
//     reg |= DIR_READ;
//     return _dev->read_registers(reg, buf, size);
// }
//
// void AP_Compass_LSM303DLHC::_register_write(uint8_t reg, uint8_t val)
// {
//     _dev->write_register(reg, val);
// }
//
// void AP_Compass_LSM303DLHC::_register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits)
// {
//     uint8_t val;
//
//     val = _register_read(reg);
//     val &= ~clearbits;
//     val |= setbits;
//     _register_write(reg, val);
// }
//
// /**
//  * Return true if the LSM303D has new data available for both the mag and
//  * the accels.
//  */
// bool AP_Compass_LSM303DLHC::_data_ready()
// {
//     return  (_register_read(ADDR_STATUS_M) & 0x01) != 0;
// }
//
//
// // Read Sensor data
// bool AP_Compass_LSM303DLHC::_read_sample()
// {
//     struct PACKED {
//         int16_t x;
//         int16_t y;
//         int16_t z;
//         uint8_t status;
//     } rx;
//
//     // if (_register_read(ADDR_CTRL_REG7) != _reg7_expected) {
//     //     hal.console->printf("LSM303D _read_data_transaction_accel: _reg7_expected unexpected\n");
//     //     return false;
//     // }
//
//     if (!_data_ready()) {
//         return false;
//     }
//
//     if (!_block_read(ADDR_OUT_X_H_M, (uint8_t *) &rx, sizeof(rx))) {
//         return false;
//     }
//
//     /* check for overrun */
//     if ((rx.status & 0x70) != 0) {
//         return false;
//     }
//
//     if (rx.x == 0 && rx.y == 0 && rx.z == 0) {
//         return false;
//     }
//
//     _mag_x = rx.x;
//     _mag_y = rx.y;
//     _mag_z = rx.z;
//
//     return true;
// }
//
// bool AP_Compass_LSM303DLHC::init(enum Rotation rotation)
// {
//     // if (LSM303D_DRDY_M_PIN >= 0) {
//     //     _drdy_pin_m = hal.gpio->channel(LSM303D_DRDY_M_PIN);
//     //     _drdy_pin_m->mode(HAL_GPIO_INPUT);
//     // }
//
//     bool success = _hardware_init();
//
//     if (!success) {
//         return false;
//     }
//
//     _initialised = true;
//
//     /* register the compass instance in the frontend */
//     _compass_instance = register_compass();
//
//     set_rotation(_compass_instance, rotation);
//
//     _dev->set_device_type(DEVTYPE_LSM303DLHC);
//     set_dev_id(_compass_instance, _dev->get_bus_id());
//
//     // read at 91Hz. We don't run at 100Hz as fetching data too fast can cause some very
//     // odd periodic changes in the output data
//     _dev->register_periodic_callback(11000, FUNCTOR_BIND_MEMBER(&AP_Compass_LSM303DLHC::_update, void));
//
//     return true;
// }
//
// bool AP_Compass_LSM303DLHC::_hardware_init()
// {
//     if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
//         AP_HAL::panic("LSM303DLHC: Unable to get semaphore");
//     }
//
//     // initially run the bus at low speed
//     _dev->set_speed(AP_HAL::Device::SPEED_LOW);
//
//     // Test WHOAMI
//     uint8_t whoami = _register_read(ADDR_WHO_AM_I);
//     if (whoami != WHO_I_AM) {
//         hal.console->printf("LSM303DLHC: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
//         goto fail_whoami;
//     }
//
//     // uint8_t tries;
//     // for (tries = 0; tries < 5; tries++) {
//     //
//
//         /* enable mag */
//         // _reg7_expected = REG7_CONT_MODE_M;
//         // _register_write(ADDR_CTRL_REG7, _reg7_expected);
//         // _register_write(ADDR_CTRL_REG5, REG5_RES_HIGH_M);
//
//         // DRDY on MAG on INT2
//         // _register_write(ADDR_CTRL_REG3, 0x08);
//
//         _mag_set_range(LSM303DLHC_MAG_DEFAULT_RANGE_GA);
//         _mag_set_samplerate(LSM303DLHC_MAG_DEFAULT_RATE);
//
//     //     hal.scheduler->delay(10);
//     //     if (_data_ready()) {
//     //         break;
//     //     }
//     // }
//     // if (tries == 5) {
//     //     hal.console->printf("Failed to boot LSM303DLHC 5 times\n");
//     //     goto fail_tries;
//     // }
//
//     _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
//     _dev->get_semaphore()->give();
//
//     return true;
//
// fail_whoami:
//     _dev->get_semaphore()->give();
//     _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
//     return false;
// }
//
// void AP_Compass_LSM303DLHC::_update()
// {
//     if (!_read_sample()) {
//         return;
//     }
//
//     Vector3f raw_field = Vector3f(_mag_x * _mag_range_scale_xy, _mag_y  * _mag_range_scale_xy, _mag_z * _mag_range_scale_z);
//
//     accumulate_sample(raw_field, _compass_instance, 10);
// }
//
// // Read Sensor data
// void AP_Compass_LSM303DLHC::read()
// {
//     if (!_initialised) {
//         return;
//     }
//
//     drain_accumulated_samples(_compass_instance);
// }
//
// bool AP_Compass_LSM303DLHC::_mag_set_range(float max_ga)
// {
//     uint8_t setbits = 0;
//     uint8_t clearbits = CRB_FULL_SCALE_BITS_M;
//     float new_scale_ga_digit_xy = 0.0f;
//     float new_scale_ga_digit_z = 0.0f;
//
//     if (max_ga == 0) {
//         max_ga = 8.1f;
//     }
//
//     if (max_ga <= 1.9f) {
//         _mag_range_ga = 1.9f;
//         setbits |= CRB_FULL_SCALE_1_9GA_M;
//         new_scale_ga_digit_xy = 100.0f/855.0f;
//         new_scale_ga_digit_z = 100.0f/760.0f;
//     } else if (max_ga <= 4.0f) {
//         _mag_range_ga = 4.0f;
//         setbits |= CRB_FULL_SCALE_4_0GA_M;
//         new_scale_ga_digit_xy = 100.0f/450.0f;
//         new_scale_ga_digit_z = 100.0f/400.0f;
//     } else if (max_ga <= 8.1f) {
//         _mag_range_ga = 8.1f;
//         setbits |= CRB_FULL_SCALE_8_1GA_M;
//         new_scale_ga_digit_xy = 100.0f/230.0f;
//         new_scale_ga_digit_z = 100.0f/205.0f;
//     } else {
//         return false;
//     }
//
//     _mag_range_scale_xy = new_scale_ga_digit_xy;
//     _mag_range_scale_z = new_scale_ga_digit_z;
//
//     _register_modify(ADDR_CRB_M, clearbits, setbits);
//
//     return true;
// }
//
// bool AP_Compass_LSM303DLHC::_mag_set_samplerate(uint16_t frequency)
// {
//     uint8_t setbits = 0;
//     uint8_t clearbits = CRA_RATE_BITS_M;
//
//     if (frequency == 0) {
//         frequency = 220;
//     }
//
//     if (frequency <= 15) {
//         setbits |= CRA_RATE_15HZ_M;
//         _mag_samplerate = 15;
//     } else if (frequency <= 30) {
//         setbits |= CRA_RATE_30HZ_M;
//         _mag_samplerate = 30;
//     } else if (frequency <= 75) {
//         setbits |= CRA_RATE_75HZ_M;
//         _mag_samplerate = 75;
//     } else if (frequency <= 220) {
//         setbits |= CRA_RATE_220HZ_M;
//         _mag_samplerate = 220;
//     } else {
//         return false;
//     }
//
//     _register_modify(ADDR_CRA_M, clearbits, setbits);
//
//     return true;
// }
