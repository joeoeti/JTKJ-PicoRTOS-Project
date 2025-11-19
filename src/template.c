
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

#define ICM42670_ADDR 0x68 // Tyypillinen I2C-osoite

// Funktio IMU:n alustukseen
void imu_init() {
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);
    // Lähetä tarvittavat init-komennot ICM-42670:lle
}

// Funktio IMU:n datan lukuun (kiihtyvyys)
void imu_read_accel(float *ax, float *ay, float *az) {
    uint8_t reg = 0x1F; // Oletus kiihtyvyysdata
    uint8_t buf[6];
    i2c_write_blocking(i2c0, ICM42670_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, ICM42670_ADDR, buf, 6, false);
    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_y = (buf[2] << 8) | buf[3];
    int16_t raw_z = (buf[4] << 8) | buf[5];
    *ax = raw_x / 16384.0f; // Skaalaus ±2g
    *ay = raw_y / 16384.0f;
    *az = raw_z / 16384.0f;
}

// Taski IMU:n lukemiseen ja napin tarkistukseen
static void imu_task(void *arg) {
    (void)arg;
    float ax, ay, az;
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    for (;;) {
        imu_read_accel(&ax, &ay, &az);
        // Lasketaan pitch ja roll
        float pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
        float roll  = atan2f(ay, az) * 180.0f / M_PI;

        if (!gpio_get(BUTTON_PIN)) { // Nappi painettu
            char symbol;
            if (pitch > 30) symbol = 'A';
            else if (pitch < -30) symbol = 'B';
            else symbol = 'C';
            printf("Symbol: %c (Pitch: %.2f, Roll: %.2f)\n", symbol, pitch, roll);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
    }
}

int main() {
    stdio_init_all();
    init_hat_sdk();
    imu_init();

    TaskHandle_t imuTaskHandle = NULL;
    xTaskCreate(imu_task, "IMU Task", DEFAULT_STACK_SIZE, NULL, 2, &imuTaskHandle);
    vTaskStartScheduler();
    return 0;
}


//Add here necessary states
enum state { IDLE=1 };
enum state programState = IDLE;

static void example_task(void *arg){
    (void)arg;

    for(;;){
        tight_loop_contents(); // Modify with application code here.
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/ 
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

