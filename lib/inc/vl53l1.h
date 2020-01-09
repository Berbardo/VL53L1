/**
 * @file vl53l1x.h
 *
 * @brief User functions to deal with ST's VL53L1 API
 *
 * @author Lucas Schneider <lucas.schneider@thunderatz.org>
 * @author Bernardo Coutinho <bernardo.coutinho@thunderatz.org>
 *
 * @date 12/2019
 */

#ifndef __VL53L1X_H__
#define __VL53L1X_H__

#include "vl53l1_api.h"
#include "vl53l1_platform.h"

/*****************************************
 * Public Constants
 *****************************************/

#define VL53L1_DEFAULT_ADDRESS 0x52

/*****************************************
 * Public Functions Prototypes
 *****************************************/

/**
 * @brief Initializes VL53L1 device
 *
 * @note Before initialization, check if the device can communicate with @ref vl53l1_wait_boot
 *
 * @param p_device      Device handler
 * @param device_info   Device informations
 * @param calibration   Device calibration data
 *
 * @return Error code
 * @retval VL53L1_ERROR_NONE   Success
 * @retval "Other error code"   More details in @ref VL53L1_Error
 */
VL53L1_Error vl53l1_init(VL53L1_Dev_t* p_device, VL53L1_DeviceInfo_t device_info,
                           VL53L1_CalibrationData_t* calibration);

/**
 * @brief Turns off device with the XSHUT pin
 *
 * @param p_device      Device handler
 */
void vl53l1_turn_off(VL53L1_Dev_t* p_device);

/**
 * @brief Turns on device with the XSHUT pin
 *
 * @param p_device      Device handler
 */
void vl53l1_turn_on(VL53L1_Dev_t* p_device);

/**
 * @brief Update sensor range
 *
 * @param p_device          Device handler
 * @param p_ranging_data    Device measurement data
 * @param range             Adress to store the measurement in mm
 * @param max_range         Max range the sensor can read in mm
 *
 * @return Error code
 * @retval 0                        Success
 * @retval "Number between 1 and 5" Range status from sensor reading @see API_User_Manual
 * @retval "Other error code"       More details in @ref VL53L1_Error
 */
uint8_t vl53l1_update_range(VL53L1_Dev_t* p_device, VL53L1_RangingMeasurementData_t* p_ranging_data, uint16_t* range,
                             uint16_t max_range);

uint8_t check_API_status();  // NOT IMPLEMENTED YET

uint8_t vl53l1_reinit();  // NOT IMPLEMENTED YET

#endif // __VL53L1X_H__
