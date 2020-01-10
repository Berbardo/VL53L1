/**
 * @file target_vl53l1_1.h
 *
 * @brief Configurations for ToFVL53l1 sensors testing board.
 *
 * @author Bernardo Coutinho <bernardo.coutinho@thunderatz.org>
 *
 * @date 12/2019
 */

#ifndef __TARGET_VL53L1_H__
#define __TARGET_VL53L1_H__

/*****************************************
 * Public Constants
 *****************************************/

/** DISTANCE SENSORS */
#define TARGET_I2C_HANDLE hi2c1
#define TARGET_I2C_INIT MX_I2C1_Init

#define FIRST_SENSOR_GPIOx GPIOA
#define FIRST_SENSOR_GPIO_PIN GPIO_PIN_6

#define SECOND_SENSOR_GPIOx GPIOA
#define SECOND_SENSOR_GPIO_PIN GPIO_PIN_5

#define THIRD_SENSOR_GPIOx GPIOA
#define THIRD_SENSOR_GPIO_PIN GPIO_PIN_7

#endif // __TARGET_VL53L1_H__
