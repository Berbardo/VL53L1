/**
 * @file main.c
 *
 * @brief Main Function.
 *
 * @author Bernardo Coutinho <bernardo.coutinho@thunderatz.org>
 *
 * @date 12/2019
 */

#include "adapter_example.h"

static uint16_t measured_distance[DS_AMOUNT];

/*****************************************
 * Main Function
 *****************************************/

int main() {
    distance_sensors_adapter_init();

    for (;;) {
        distance_sensors_adapter_update();

        for (distance_sensor_position_t sensor_position = DS_FRONT_LEFT; sensor_position < DS_AMOUNT; sensor_position++) {
            measured_distance[(int) sensor_position] = distance_sensors_adapter_get(sensor_position);
        }
    }
}
