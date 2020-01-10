/**
 * @file adapter_example.h
 *
 * @brief Adapter example code for VL53L1 ToF sensors.
 *
 * @author Lucas Schneider <lucas.schneider@thunderatz.org>
 * @author Bernardo Coutinho <bernardo.coutinho@thunderatz.org>
 *
 * @date 11/2019
 */

#ifndef __ADAPTER_EXAMPLE_H__
#define __ADAPTER_EXAMPLE_H__

#include <stdint.h>
#include <stdbool.h>

/*****************************************
 * Public Constants
 *****************************************/

/**
 * @brief Enumerates distance sensors.
 */
typedef enum _distance_sensor_position {
    DS_FRONT_LEFT = 0,
    DS_FRONT_CENTER = 1,
    DS_FRONT_RIGHT = 2,

    DS_AMOUNT = 3
} distance_sensor_position_t;

/*****************************************
 * Public Functions Prototypes
 *****************************************/

/**
 * @brief Initializes distance sensors adapter.
 */
uint8_t distance_sensors_adapter_init();

/**
 * @brief Updates distance sensor readings.
 */
uint8_t distance_sensors_adapter_update();

/**
 * @brief Returns readings from specified distance sensor.
 *
 * @param sensor Specified sensor.
 */
uint16_t distance_sensors_adapter_get(distance_sensor_position_t sensor);

void vl53l1_shield_control(distance_sensor_position_t sensor, uint8_t state);

#endif // __ADAPTER_EXAMPLE_H__
