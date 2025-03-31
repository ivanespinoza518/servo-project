#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_log.h>

#include <driver/mcpwm_prelude.h>

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>

#include <driver/i2c_master.h>
#include <driver/i2c.h>
#include "i2c-lcd.h"

// Counter parameters
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1Mhz
#define SERVO_TIMEBASE_PERIOD        20000   // 20ms

// Servo Motor aspects
#define SERVO_MIN_WIDTH_US 500  // fully clockwise (looking at horn)
#define SERVO_MAX_WIDTH_US 2500 // fully counter clockwise (looking at horn)
#define SERVO_SIGNAL_PIN   15

#define POT_MAX 4095

#define POT_GND GPIO_NUM_32
#define POT_VCC GPIO_NUM_27
#define POT_SIG GPIO_NUM_25

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

QueueHandle_t servoqueue;
QueueHandle_t displayqueue;

adc_oneshot_unit_handle_t adc_handle;

void setUpPot() {
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id  = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_8, &chan_config));
}

static inline int potToPWM(int potvalue) {
    int result = potvalue * (SERVO_MAX_WIDTH_US - SERVO_MIN_WIDTH_US) / POT_MAX + SERVO_MIN_WIDTH_US;
    // printf("pot value = %d, result = %d\n", potvalue, result);
    return result;
}

void servoTask(void *arg) {
    // Create timer

    // counter
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    mcpwm_timer_handle_t timer;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // operator engine to apply some sort of function to the counter
    mcpwm_operator_config_t op_config = {
        .group_id = 0
    };
    mcpwm_oper_handle_t operator;
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &operator));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator, timer));

    // comparison
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true
    };
    mcpwm_cmpr_handle_t comparator;
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &comparator_config, &comparator));

    // pin output generator
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_SIGNAL_PIN
    };
    mcpwm_gen_handle_t generator;
    ESP_ERROR_CHECK(mcpwm_new_generator(operator, &generator_config, &generator));
    
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, potToPWM(0)));

    // go high on counter empty (zero)
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
    ));

    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)
    ));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    int value;
    while (1) {
        xQueueReceive(servoqueue, &value, portMAX_DELAY);
        mcpwm_comparator_set_compare_value(comparator, potToPWM(value));
    }
}

static const char *TAG = "i2c-simple-example";

static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    i2c_param_config(i2c_master_port, &conf);
    
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void displayTask(void *arg) {
    // do setup needed for display
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized successfully");

    lcd_init();

    int value;
    while (1) {
        xQueueReceive(displayqueue, &value, portMAX_DELAY);

        // write value to screen
        lcd_clear();
        char str[16];

        lcd_put_cur(0, 0);
        snprintf(str, sizeof(str), "POT:%8d", value);
        lcd_send_string(str);

        int servo_value = potToPWM(value);
        lcd_put_cur(1, 0);
        snprintf(str, sizeof(str), "SERVO:%6d", servo_value);
        lcd_send_string(str);
    }
}

// fully clockwise ... comparator setpoint = 500
void app_main() {
    servoqueue = xQueueCreate(10, sizeof(int *));
    displayqueue = xQueueCreate(10, sizeof(int *));

    gpio_set_direction(POT_GND, GPIO_MODE_OUTPUT);
    gpio_set_level(POT_GND, 0);

    gpio_set_direction(POT_VCC, GPIO_MODE_OUTPUT);
    gpio_set_level(POT_VCC, 1);

    setUpPot();

    int potvalue = -1;

    TaskHandle_t servo_handle;
    xTaskCreate(servoTask, "Servo Task", 4096, NULL, 5, &servo_handle);

    TaskHandle_t display_handle;
    xTaskCreate(displayTask, "Display Task", 4096, NULL, 8, &display_handle);

    while(1) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_8, &potvalue);

        xQueueSend(servoqueue, &potvalue, portMAX_DELAY);
        xQueueSend(displayqueue, &potvalue, portMAX_DELAY);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}