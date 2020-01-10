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

#define INIT_RESET_SLEEP_TIME_MS 10
#define MAX_RANGE_MM 4000

/*****************************************
 * Private Variables
 *****************************************/

static VL53L1_Dev_t sensors[] = {
    { // 0
        .I2cDevAddr = VL53L1_DEFAULT_ADDRESS,
        .comms_type = 1, // I2C
        .comms_speed_khz = VL53L1_DEFAULT_COMM_SPEED_KHZ,
        .present = 0,
        .calibrated = 0,
        .I2cHandle = &TARGET_I2C_HANDLE,
        .xshut_port = FIRST_SENSOR_GPIOx,
        .xshut_pin = FIRST_SENSOR_GPIO_PIN
    },
    { // 1
        .I2cDevAddr = VL53L1_DEFAULT_ADDRESS,
        .comms_type = 1, // I2C
        .comms_speed_khz = VL53L1_DEFAULT_COMM_SPEED_KHZ,
        .present = 0,
        .calibrated = 0,
        .I2cHandle = &TARGET_I2C_HANDLE,
        .xshut_port = SECOND_SENSOR_GPIOx,
        .xshut_pin = SECOND_SENSOR_GPIO_PIN
    },
    { // 2
        .I2cDevAddr = VL53L1_DEFAULT_ADDRESS,
        .comms_type = 1, // I2C
        .comms_speed_khz = VL53L1_DEFAULT_COMM_SPEED_KHZ,
        .present = 0,
        .calibrated = 0,
        .I2cHandle = &TARGET_I2C_HANDLE,
        .xshut_port = THIRD_SENSOR_GPIOx,
        .xshut_pin = THIRD_SENSOR_GPIO_PIN
    }
};

static VL53L1_DeviceInfo_t sensors_info[DS_AMOUNT];
static VL53L1_RangingMeasurementData_t sensors_measurement[DS_AMOUNT];
static VL53L1_CalibrationData_t sensors_calibration[DS_AMOUNT];

static uint16_t actual_range[] = {MAX_RANGE_MM, MAX_RANGE_MM, MAX_RANGE_MM};
static const uint8_t used_sensors[] = {1, 1, 0};
static const uint8_t i2c_addresses[] = {0x30, 0x34, 0x38};
__attribute__((used)) static uint8_t sensors_status[] = {0, 0, 0};

/*****************************************
 * Public Functions Bodies Definitions
 *****************************************/

uint8_t distance_sensors_adapter_init(void) {
    TARGET_I2C_INIT();

    VL53L1_Error global_status = VL53L1_ERROR_NONE;

    // desabilita todos, independente de quantos vai usar
    for (int i = 0; i < DS_AMOUNT; i++) {
        vl53l1_turn_off(&(sensors[i]));

        // vl53l1_shield_control(i, 0);
    }

    mcu_sleep(INIT_RESET_SLEEP_TIME_MS);

    for (int i = 0; i < DS_AMOUNT; i++) {
        if (!used_sensors[i]) {
            continue;
        }

        VL53L1_Error status = VL53L1_ERROR_NONE;
        VL53L1_Dev_t* p_device = &(sensors[i]);

        // vl53l1_turn_on(p_device);
        // uint16_t wordData;
        // VL53L1_RdWord(p_device, VL53L1_IDENTIFICATION__MODEL_ID, &wordData);

        // vl53l1_shield_control(i, 1);
        status = vl53l1_wait_boot(p_device); // Trocar pelo turn on depois, peguei essa função da lib do vl53l0x mas teoricamente ela não é mais necessária

        if (status == VL53L1_ERROR_NONE) {
            status = VL53L1_SetDeviceAddress(p_device, i2c_addresses[i]);
        }

        if (status == VL53L1_ERROR_NONE) {
            p_device->I2cDevAddr = i2c_addresses[i];
            status = vl53l1_init(p_device, sensors_info[i], &sensors_calibration[i]);
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
            vl53l1_update_range(&(sensors[i]), &(sensors_measurement[i]), &(actual_range[i]), MAX_RANGE_MM);

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

/*****************************************
 * Shield Specific Constants
 *****************************************/

/**
 * Expander 0 i2c address[7..0] format
 */
#define I2cExpAddr0 ((int) (0x43 * 2))

/**
 * Expander 1 i2c address[7..0] format
 */
#define I2cExpAddr1 ((int) (0x42 * 2))

/** @} XNUCLEO53L1A1_I2CExpanders */

/**
 * GPIO monitor pin state register
 * 16 bit register LSB at lowest offset (little endian)
 */
#define GPMR 0x10

/**
 * STMPE1600 GPIO set pin state register
 * 16 bit register LSB at lowest offset (little endian)
 */
#define GPSR 0x12

/**
 * STMPE1600 GPIO set pin direction register
 * 16 bit register LSB at lowest offset
 */
#define GPDR 0x14

/*****************************************
 * Shield Specific Function Definition
 *****************************************/

void vl53l1_shield_control(distance_sensor_position_t sensor, uint8_t state) {
    uint8_t Rd_RegAddr = GPMR + 1;
    uint8_t data;
    uint8_t Wr_RegAddr[0x10];
    Wr_RegAddr[0] = GPSR + 1;

    switch (sensor) {
        case DS_FRONT_CENTER: {
            HAL_I2C_Master_Transmit(&TARGET_I2C_HANDLE, I2cExpAddr1, &Rd_RegAddr, 1, 100);
            HAL_I2C_Master_Receive(&TARGET_I2C_HANDLE, I2cExpAddr1, &data, 1, 100);
            data &= ~0x80;

            if (state) {
                data |= 0x80;
            }

            Wr_RegAddr[1] = data;
            HAL_I2C_Master_Transmit(&TARGET_I2C_HANDLE, I2cExpAddr1, Wr_RegAddr, 2, 100);
            break;
        }

        case DS_FRONT_LEFT: {
            HAL_I2C_Master_Transmit(&TARGET_I2C_HANDLE, I2cExpAddr0, &Rd_RegAddr, 1, 100);
            HAL_I2C_Master_Receive(&TARGET_I2C_HANDLE, I2cExpAddr0, &data, 1, 100);
            data &= ~0x40;

            if (state) {
                data |= 0x40;
            }

            Wr_RegAddr[1] = data;
            HAL_I2C_Master_Transmit(&TARGET_I2C_HANDLE, I2cExpAddr0, Wr_RegAddr, 2, 100);
            break;
        }

        case DS_FRONT_RIGHT: {
            HAL_I2C_Master_Transmit(&TARGET_I2C_HANDLE, I2cExpAddr0, &Rd_RegAddr, 1, 100);
            HAL_I2C_Master_Receive(&TARGET_I2C_HANDLE, I2cExpAddr0, &data, 1, 100);
            data &= ~0x80;

            if (state) {
                data |= 0x80;
            }

            Wr_RegAddr[1] = data;
            HAL_I2C_Master_Transmit(&TARGET_I2C_HANDLE, I2cExpAddr0, Wr_RegAddr, 2, 100);
            break;
        }
    }
}
