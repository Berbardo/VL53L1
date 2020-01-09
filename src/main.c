/**
 * @file main.c
 *
 * @brief Main Function.
 *
 * @author Bernardo Coutinho <bernardo.coutinho@thunderatz.org>
 *
 * @date 12/2019
 */

#include <stdio.h>

#include "adapter_example.h"

#include "mcu.h"

__attribute__((used)) static uint16_t measured_distance[DS_AMOUNT];

/*****************************************
 * Main Function
 *****************************************/

int main() {
    mcu_init();

    mcu_sleep(300);

    distance_sensors_adapter_init();

    printf("************ START TESTS ************\r\n");

    for (;;) {
        distance_sensors_adapter_update();

        for (distance_sensor_position_t sensor_position = DS_FRONT_LEFT; sensor_position < DS_AMOUNT; sensor_position++) {
            measured_distance[(int) sensor_position] = distance_sensors_adapter_get(sensor_position);

            printf("Sensor %d value: %d\r\n", sensor_position, measured_distance[(int) sensor_position]);
        }

        mcu_sleep(1000);
        printf("\r\n");
    }
}
