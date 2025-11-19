
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#include <math.h>
#include "hardware/i2c.h"

// Stack size
#define DEFAULT_STACK_SIZE 2048 

int init_ICM42670(void);
int start_sensor_with_default_values(void);
int ICM42670_read_sensor_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *t);

// Taski IMU:n lukemiseen ja napin tarkistukseen
static void imu_task(void *arg) {
    (void)arg;
    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    const float dt = 0.05f; // 50 ms, same as taskDelay

    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_pull_up(BUTTON1);

    for (;;) {
        ICM42670_read_sensor_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *t); // Gyro sensor read °/s
        // integrate time
        pitch += gx * dt;
        roll  += gy * dt;
        yaw   += gz * dt;

        if (!gpio_get(BUTTON1)) { // Nappi painettu
            char symbol;
            if (pitch > 30) symbol = 'A';
            else if (pitch < -30) symbol = 'B';
            else symbol = 'C';
            printf("Symbol: %c (Pitch: %.2f, Roll: %.2f, Yaw: %.2f)\n", symbol, pitch, roll, yaw);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms, same as time for gyro integral
    }
}

int main() {
    stdio_init_all();
    // uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(300); // wait some time so initialization of USB and hat is done.
    imu_init(); // initialize IMU

    TaskHandle_t imuTaskHandle = NULL;
    xTaskCreate(imu_task, "IMU Task", DEFAULT_STACK_SIZE, NULL, 2, &imuTaskHandle);
    vTaskStartScheduler();
    return 0;
}


// add here necessary states
enum state { IDLE=1 };
enum state programState = IDLE;

static void example_task(void *arg){
    (void)arg;

    for(;;){
        tight_loop_contents(); // modify with application code here.
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    TaskHandle_t myExampleTask = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(example_task,       // (en) Task function
                "example",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &myExampleTask);    // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Example Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

