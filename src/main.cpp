#include <stdint.h>
#include <stdio.h>

#include "pico/binary_info.h"
#include "pico/stdlib.h"

extern "C" {
#include "VL53L1X_api.h"
#include "VL53L1X_types.h"
}

#define I2C_DEV_ADDR 0x29

#define RETURN_PIN 15

VL53L1X_Status_t status;
VL53L1X_Result_t results;
uint16_t default_distance;
bool hasDefaultDistance = false;
bool triggered = false;

int main() {
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_init(RETURN_PIN);
    gpio_set_dir(RETURN_PIN, GPIO_OUT);

    // Initialize Pico's I2C using PICO_DEFAULT_I2C_SDA_PIN
    // and PICO_DEFAULT_I2C_SCL_PIN (GPIO 4 and GPIO 5, respectively)
    if (VL53L1X_I2C_Init(I2C_DEV_ADDR, i2c0) < 0) {
        printf("Error initializing sensor.\n");
        return 0;
    }

    // Ensure the sensor has booted
    uint8_t sensorState;
    do {
        status += VL53L1X_BootState(I2C_DEV_ADDR, &sensorState);
        VL53L1X_WaitMs(I2C_DEV_ADDR, 2);
    } while (sensorState == 0);
    printf("Sensor booted.\n");

    // Initialize and configure sensor
    status = VL53L1X_SensorInit(I2C_DEV_ADDR);
    status += VL53L1X_SetDistanceMode(I2C_DEV_ADDR, 1);
    status += VL53L1X_SetTimingBudgetInMs(I2C_DEV_ADDR, 100);
    status += VL53L1X_SetInterMeasurementInMs(I2C_DEV_ADDR, 100);
    status += VL53L1X_StartRanging(I2C_DEV_ADDR);

    // Measure and print continuously
    bool first_range = true;
    while (1) {
        // Wait until we have new data
        uint8_t dataReady;
        do {
            status = VL53L1X_CheckForDataReady(I2C_DEV_ADDR, &dataReady);
            sleep_us(1);
        } while (dataReady == 0);

        // Read and display result
        status += VL53L1X_GetResult(I2C_DEV_ADDR, &results);
        printf("Status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n", results.status, results.distance, results.ambient, results.sigPerSPAD, results.numSPADs);

        if (results.distance == 0) {
            // Clear the sensor for a new measurement
            status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
            if (first_range) {  // Clear twice on first measurement
                status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
                first_range = false;
            }
            continue;
        }

        if (!hasDefaultDistance) {
            default_distance = results.distance;
            hasDefaultDistance = true;
        }

        if (results.distance < default_distance - 150) {
            if (!triggered) {
                gpio_put(PICO_DEFAULT_LED_PIN, 1);
                gpio_put(RETURN_PIN, 1);
                triggered = true;
                printf("Triggered!\n");
            }
        } else if (triggered) {
            gpio_put(RETURN_PIN, 0);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            triggered = false;
            printf("Reset.\n");
        }

        // Clear the sensor for a new measurement
        status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
        if (first_range) {  // Clear twice on first measurement
            status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
            first_range = false;
        }
    }

    while (1) {
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(250);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(1000);
    }
}