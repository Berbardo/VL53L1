/**
 * @file adapter_example.c
 *
 * @brief Adapter example code for VL53L1 ToF sensors.
 *
 * @author Lucas Schneider <lucas.schneider@thunderatz.org>
 * @author Bernardo Coutinho <bernardo.coutinho@thunderatz.org>
 *
 * @date 01/2020
 */

#include <stdbool.h>
#include <stdint.h>

#include "adapter_example.h"

#include "vl53l1.h"

#include "mcu.h"
#include "main.h"
#include "i2c.h"

/*****************************************
 * Private Constants
 *****************************************/

#define VL53L1_DEFAULT_COMM_SPEED_KHZ 100
#define TIMING_BUDGET_US 50000

#define INIT_RESET_SLEEP_TIME_MS 10
#define MAX_RANGE_MM 4000

/*****************************************
 * Private Variables
 *****************************************/

static VL53L1_Dev_t sensors[] = {
    { // 0
        .comms_speed_khz = VL53L1_DEFAULT_COMM_SPEED_KHZ,
        .I2cHandle = &TARGET_I2C_HANDLE,
        .xshut_port = FIRST_SENSOR_GPIOx,
        .xshut_pin = FIRST_SENSOR_GPIO_PIN
    },
    { // 1
        .comms_speed_khz = VL53L1_DEFAULT_COMM_SPEED_KHZ,
        .I2cHandle = &TARGET_I2C_HANDLE,
        .xshut_port = SECOND_SENSOR_GPIOx,
        .xshut_pin = SECOND_SENSOR_GPIO_PIN
    },
    { // 2
        .comms_speed_khz = VL53L1_DEFAULT_COMM_SPEED_KHZ,
        .I2cHandle = &TARGET_I2C_HANDLE,
        .xshut_port = THIRD_SENSOR_GPIOx,
        .xshut_pin = THIRD_SENSOR_GPIO_PIN
    }
};

static VL53L1_RangingMeasurementData_t sensors_measurement[DS_AMOUNT];
static VL53L1_CalibrationData_t sensors_calibration[DS_AMOUNT];

static uint16_t actual_range[] = {MAX_RANGE_MM, MAX_RANGE_MM, MAX_RANGE_MM};
static const uint8_t used_sensors[] = {1, 0, 1};
static const uint8_t i2c_addresses[] = {0x30, 0x34, 0x38};
__attribute__((used)) static uint8_t sensors_status[] = {0, 0, 0};

static const VL53L1_DistanceModes sensor_distance_mode[] = {VL53L1_DISTANCEMODE_MEDIUM, VL53L1_DISTANCEMODE_MEDIUM, VL53L1_DISTANCEMODE_MEDIUM};
static const uint32_t sensor_timing_budget_us[] = {TIMING_BUDGET_US, TIMING_BUDGET_US, TIMING_BUDGET_US};

/*****************************************
 * Public Functions Bodies Definitions
 *****************************************/

uint8_t distance_sensors_adapter_init(void) {
    TARGET_I2C_INIT();

    VL53L1_Error global_status = VL53L1_ERROR_NONE;

    // desabilita todos, independente de quantos vai usar
    for (int i = 0; i < DS_AMOUNT; i++) {
        vl53l1_turn_off(&(sensors[i]));
    }

    mcu_sleep(INIT_RESET_SLEEP_TIME_MS);

    for (int i = 0; i < DS_AMOUNT; i++) {
        if (!used_sensors[i]) {
            continue;
        }

        VL53L1_Error status = VL53L1_ERROR_NONE;
        VL53L1_Dev_t* p_device = &(sensors[i]);

        vl53l1_set_default_config(p_device);
        p_device->distance_mode = sensor_distance_mode[i];
        p_device->timing_budget_us = sensor_timing_budget_us[i];

        vl53l1_turn_on(p_device);

        if (status == VL53L1_ERROR_NONE) {
            status = VL53L1_SetDeviceAddress(p_device, i2c_addresses[i]);
        }

        if (status == VL53L1_ERROR_NONE) {
            p_device->I2cDevAddr = i2c_addresses[i];
            status = vl53l1_init(p_device, &sensors_calibration[i]);
        }

        if (status == VL53L1_ERROR_NONE) {
            p_device->present = 1;
            p_device->calibrated = 1;
        }

        global_status |= status;
    }

    if (global_status == VL53L1_ERROR_NONE) {
        return 0;
    }

    return 1;
}

uint8_t distance_sensors_adapter_update(void) {
    uint8_t status = 0;

    for (int i = 0; i < DS_AMOUNT; i++) {
        if (!used_sensors[i]) {
            continue;
        }

        sensors_status[i] =
            vl53l1_update_reading(&(sensors[i]), &(sensors_measurement[i]), &(actual_range[i]), MAX_RANGE_MM);

        if (sensors_status[i] != 0) {
            status |= 1 << (i + 1);
        }
    }

    return status;
}

uint16_t distance_sensors_adapter_get(distance_sensor_position_t sensor) {
    if ((sensors[(int) sensor]).present) {
        return actual_range[(int) sensor];
    }

    return -1;
}
