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
#define I2C_HANDLE &hi2c1
#define TARGET_I2C_INIT MX_I2C1_Init

#define FIRST_SENSOR_GPIOx GPIOC
#define FIRST_SENSOR_GPIO_PIN GPIO_PIN_14

#define SECOND_SENSOR_GPIOx GPIOC
#define SECOND_SENSOR_GPIO_PIN GPIO_PIN_0

#define THIRD_SENSOR_GPIOx GPIOC
#define THIRD_SENSOR_GPIO_PIN GPIO_PIN_2

#endif // __TARGET_VL53L1_H__
