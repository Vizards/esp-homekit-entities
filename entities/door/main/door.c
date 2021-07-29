// 角度传感器小于 2300 认为已经完全打开，大于 3100 认为已经完全关上

#include <stdio.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "wifi.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "string.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define POSITION_OPEN 100
#define POSITION_CLOSED 0
#define POSITION_STATE_CLOSING 0
#define POSITION_STATE_OPENING 1
#define POSITION_STATE_STOPPED 2
#define OBSTRUCTION_DETECTED true
#define OBSTRUCTION_NOT_DETECTED false

TaskHandle_t updateStateTask;
homekit_characteristic_t current_position;
homekit_characteristic_t target_position;
homekit_characteristic_t position_state;
homekit_characteristic_t obstruction_detected;
homekit_accessory_t *accessories[];
const int open_gpio = 12;
const int close_gpio = 14;
const int sensor_vin = 5;
static const adc_atten_t atten = ADC_ATTEN_11db; // 满量程为3.9v
static const adc_channel_t channel_angle = ADC_CHANNEL_6; // GPIO34
bool is_opening_or_closing = false;
bool is_processing_homekit_instruction = false;
void on_wifi_ready();

esp_err_t event_handler(void *ctx, system_event_t *event)
{
        switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
                printf("STA start\n");
                esp_wifi_connect();
                break;
        case SYSTEM_EVENT_STA_GOT_IP:
                printf("WiFI ready\n");
                on_wifi_ready();
                break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
                printf("STA disconnected\n");
                esp_wifi_connect();
                break;
        default:
                break;
        }
        return ESP_OK;
}

static void wifi_init() {
        tcpip_adapter_init();
        ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

        wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

        wifi_config_t wifi_config = {
                .sta = {
                        .ssid = WIFI_SSID,
                        .password = WIFI_PASSWORD,
                },
        };

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
}

// min = 0 = 关上; max = 100 = 打开;
int get_door_ratio() {
    int angle_ratio = 0;
    int adc_reading = 0;
    adc1_config_width(ADC_WIDTH_BIT_12); //采集宽度
    adc1_config_channel_atten(channel_angle, atten); //配置通道 以及衰减度

    printf("开始中值滤波\n");
    const int FILTER_N = 11;
    int filter_buf[FILTER_N];
    int i, j;
    int filter_temp;
    for (i = 0; i < FILTER_N; i++) {
        filter_buf[i] = adc1_get_raw((adc1_channel_t)channel_angle);
        vTaskDelay(1);
    }
    // 采样值从小到大排列
    for (j = 0; j < FILTER_N; j++) {
        for (i = 0; i < FILTER_N - 1 -j; i++) {
            if (filter_buf[i] > filter_buf[i + 1]) {
                filter_temp = filter_buf[i];
                filter_buf[i] = filter_buf[i + 1];
                filter_buf[i + 1] = filter_temp;
            }
        }
    }

    adc_reading = filter_buf[(FILTER_N - 1) / 2];
    if (adc_reading > 3100) {
        angle_ratio = 0;
    } else if (adc_reading < 2400) {
        angle_ratio = 100;
    } else {
        angle_ratio = (3100 - adc_reading) / 10;
    }
    return angle_ratio;
}

void open_door(uint32_t target_position) {
    printf("接收到开门处理信号，目标位置：%d\n", target_position);
    if (is_opening_or_closing == true) {
        printf("当前正在开门或关门，不接受重复命令\n");
        return;
    }
    printf("正在初始化变量...\n");
    is_opening_or_closing = true;
    int current_door_ratio = 0;
    int the_times_that_current_position_equal_to_last_position = 0;
    int last_current_door_ratio = current_door_ratio;
    
    printf("重置 HOMEKIT 状态为未卡住\n");
    obstruction_detected.value.bool_value = OBSTRUCTION_NOT_DETECTED;
    homekit_characteristic_notify(&obstruction_detected, obstruction_detected.value);
    
    printf("处理 GPIO 引脚事件\n");
    gpio_set_level(open_gpio, 1);
    
    printf("通知 HOMEKIT 更新为开启中状态\n");
    position_state.value.int_value = POSITION_STATE_OPENING;
    homekit_characteristic_notify(&position_state, position_state.value);
    
    printf(".......等待角度传感器回传位置信号.....\n");
    while(1) {
        current_door_ratio = get_door_ratio();
        if (current_door_ratio >= target_position) {
            printf("已经打开到指定位置\n");
            if (target_position == 100) {
                printf("多运行一会儿以保证开到位\n");
                vTaskDelay(200);
            }
            break;
        }
        if (current_door_ratio < target_position) {
            printf("正在打开中\n");
            if (current_door_ratio - last_current_door_ratio < 10) {
                the_times_that_current_position_equal_to_last_position++;
                printf("认为几乎没有旋转，次数加 1，当前次数 %d\n", the_times_that_current_position_equal_to_last_position);
            } else {
                printf("认为旋转了一定角度，清空次数\n");
                the_times_that_current_position_equal_to_last_position = 0;
            }
        }
        if (the_times_that_current_position_equal_to_last_position >= 50) {
            printf("连续 %d 次没有旋转，认为卡住，结束旋转\n", the_times_that_current_position_equal_to_last_position);
            obstruction_detected.value.bool_value = OBSTRUCTION_DETECTED;
            homekit_characteristic_notify(&obstruction_detected, obstruction_detected.value);
            break;
        }
        last_current_door_ratio = current_door_ratio;
        vTaskDelay(50);
    }
    
    printf("开启完毕，恢复 GPIO 引脚\n");
    gpio_set_level(open_gpio, 0);
    
    printf("通知 HOMEKIT 更新为停止状态\n");
    position_state.value.int_value = POSITION_STATE_STOPPED;
    homekit_characteristic_notify(&position_state, position_state.value);
    current_position.value.int_value = current_door_ratio;
    homekit_characteristic_notify(&current_position, current_position.value);

    vTaskDelay(100);
    is_opening_or_closing = false;
    printf("开门操作全部结束\n");
}

void close_door(int target_position) {
    printf("接收到关门处理信号，目标位置 %d\n", target_position);
    if (is_opening_or_closing == true) {
        printf("当前正在开门或关门，不接受重复命令\n");
        return;
    }
    
    printf("正在初始化变量...\n");
    is_opening_or_closing = true;
    int current_door_ratio = 0;
    int the_times_that_current_position_equal_to_last_position = 0;
    int last_current_door_ratio = current_door_ratio;

    printf("重置 HOMEKIT 状态为未卡住\n");
    obstruction_detected.value.bool_value = OBSTRUCTION_NOT_DETECTED;
    homekit_characteristic_notify(&obstruction_detected, obstruction_detected.value);
    
    printf("处理 GPIO 引脚事件\n");
    gpio_set_level(close_gpio, 1);
    
    printf("通知 HOMEKIT 更新为关闭中状态\n");
    position_state.value.int_value = POSITION_STATE_CLOSING;
    homekit_characteristic_notify(&position_state, position_state.value);

    printf(".......等待角度传感器回传位置信号.....\n");
    while(1) {
        current_door_ratio = get_door_ratio();
        if (current_door_ratio - target_position < 8) {
            printf("已经关闭到指定位置\n");
            if (target_position == 0) {
                printf("多运行一会儿以保证关到位\n");
                vTaskDelay(200);
            }
            break;
        }
        if (current_door_ratio > target_position) {
            printf("正在关闭中\n");
            if (last_current_door_ratio - current_door_ratio < 10) {
                the_times_that_current_position_equal_to_last_position++;
                printf("认为几乎没有旋转，次数加 1，当前次数 %d\n", the_times_that_current_position_equal_to_last_position);
            } else {
                printf("认为旋转了一定角度，清空次数\n");
                the_times_that_current_position_equal_to_last_position = 0;
            }
        }
        if (the_times_that_current_position_equal_to_last_position >= 50) {
            printf("连续 %d 次没有旋转，认为卡住，结束旋转\n", the_times_that_current_position_equal_to_last_position);
            obstruction_detected.value.bool_value = OBSTRUCTION_DETECTED;
            homekit_characteristic_notify(&obstruction_detected, obstruction_detected.value);
            break;
        }
        last_current_door_ratio = current_door_ratio;
        vTaskDelay(50);
    }
    
    printf("关闭完毕，恢复 GPIO 引脚\n");
    gpio_set_level(close_gpio, 0);
    
    printf("通知 HOMEKIT 更新为停止状态\n");
    position_state.value.int_value = POSITION_STATE_STOPPED;
    homekit_characteristic_notify(&position_state, position_state.value);
    current_position.value.int_value = target_position;
    homekit_characteristic_notify(&current_position, current_position.value);

    vTaskDelay(100);
    is_opening_or_closing = false;
    printf("关门操作全部结束\n");
}

// 监听 HomeKit 事件，不需要一直监听
void update_state() {
    while (true) {        
        is_processing_homekit_instruction = true;
        printf("当前位置: %u\n", current_position.value.int_value);
        printf("目标位置: %u\n", target_position.value.int_value);
        if (target_position.value.int_value - current_position.value.int_value >= 10) {
            open_door(target_position.value.int_value);
        }
        if (target_position.value.int_value - current_position.value.int_value <= -10) {
            close_door(target_position.value.int_value);
        }
        is_processing_homekit_instruction = false;
        vTaskSuspend(updateStateTask);
    }
}

// 监听当前角度并上报给 HomeKit
void update_current_state() {
    while (true) {
        vTaskDelay(200);
        if (is_opening_or_closing == false && is_processing_homekit_instruction == false) {
            int current_door_ratio = get_door_ratio();
            if (current_door_ratio < 10 && current_position.value.int_value != 0) {
                printf("在检测到手动关门时，自动将门关严\n");
                close_door(0);
                continue;
            }
            if (current_door_ratio - current_position.value.int_value > 6 || current_position.value.int_value - current_door_ratio > 6) {
                printf("移动角度大于百分之六，才上报消息\n");
                if (current_door_ratio < 10) {
                    current_door_ratio = 0;
                }
                position_state.value.int_value = POSITION_STATE_STOPPED;
                current_position.value.int_value = current_door_ratio;
                target_position.value.int_value = current_door_ratio;
                homekit_characteristic_notify(&current_position, current_position.value);
                homekit_characteristic_notify(&target_position, target_position.value);
                homekit_characteristic_notify(&position_state, position_state.value);
            }
        }
    }
    
}

void update_state_init() {
    xTaskCreate(update_current_state, "UpdateCurrentState", 4096, NULL, 5, NULL);
    xTaskCreate(update_state, "UpdateState", 4096, NULL, tskIDLE_PRIORITY, &updateStateTask);
    vTaskSuspend(updateStateTask);
}

void door_identify(homekit_value_t _value) {
    printf("Door identify\n");
}

void on_update_target_position(homekit_characteristic_t *ch, homekit_value_t value, void *context);

homekit_value_t gdo_current_state_get();

homekit_characteristic_t current_position = {
    HOMEKIT_DECLARE_CHARACTERISTIC_CURRENT_POSITION(POSITION_CLOSED)
};

homekit_characteristic_t target_position = {
    HOMEKIT_DECLARE_CHARACTERISTIC_TARGET_POSITION(POSITION_CLOSED, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update_target_position))
};

homekit_characteristic_t position_state = {
    HOMEKIT_DECLARE_CHARACTERISTIC_POSITION_STATE(POSITION_STATE_STOPPED)
};

homekit_characteristic_t obstruction_detected = {
    HOMEKIT_DECLARE_CHARACTERISTIC_OBSTRUCTION_DETECTED(OBSTRUCTION_NOT_DETECTED)
};


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_door, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Door"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "MNK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "001"),
            HOMEKIT_CHARACTERISTIC(MODEL, "MyDoor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, door_identify),
            NULL
        }),
        HOMEKIT_SERVICE(DOOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Door"),
            &current_position,
            &target_position,
            &position_state,
            &obstruction_detected,
            NULL
        }),
        NULL
    }),
    NULL
};

void on_update_target_position(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
    printf("HomeKit 要求更新目标位置到: %u\n", target_position.value.int_value);
    vTaskResume(updateStateTask);
}

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void control_init() {
    printf("初始化正反电流控制器引脚\n");
    gpio_reset_pin(open_gpio);
    gpio_reset_pin(close_gpio);
    gpio_set_direction(open_gpio, GPIO_MODE_OUTPUT);
    gpio_set_direction(close_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(open_gpio, 0);
    gpio_set_level(close_gpio, 0);
    
    printf("初始化传感器供电\n");
    gpio_set_direction(sensor_vin, GPIO_MODE_OUTPUT);
    gpio_set_level(sensor_vin, 1);

    printf("控制系统初始化完成\n");
}

void on_wifi_ready() {
    printf("WiFi 初始化完成\n");
    homekit_server_init(&config);
    control_init();
    update_state_init();
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
}
