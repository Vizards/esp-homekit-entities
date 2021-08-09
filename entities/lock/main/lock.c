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
#include "string.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"

typedef enum {
    lock_state_unsecured = 0,
    lock_state_secured = 1,
    lock_state_jammed = 2,
    lock_state_unknown = 3,
} lock_state_t;

TaskHandle_t updateStateTask;
const int doorbell_gpio = 5;
const int lock_gpio = 31;
void on_wifi_ready();

esp_err_t event_handler(void *ctx, system_event_t *event) {
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
};

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
};


void update_doorbell_state() {
    while (true){
        int gpio_level_doorbell = gpio_get_level(doorbell_gpio);
        if (gpio_level_doorbell == 1) {
            homekit_characteristic_notify(&button_event, HOMEKIT_UINT8(0));
        }
    }
}

void lock_lock() {
    lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
}

void lock_unlock() {
    printf("接收到开锁命令\n");
    printf("处理 GPIO 引脚事件\n");
    gpio_set_level(lock_gpio, 1);
    lock_current_state.value = HOMEKIT_UINT8(lock_state_unsecured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);

    sleep(3);
    printf("等待 3s 自动锁上 \n");
    lock_target_state.value = HOMEKIT_UINT8(lock_state_secured);
    lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
}

void lock_target_state_setter(homekit_value_t value) {
    lock_target_state.value = value;
    if (value.int_value == 0) {
        lock_unlock();
    } else {
        lock_lock();
    }
};
void lock_identify(homekit_value_t _value) {
    printf("Lock identify\n");
};
void doorbell_identify(homekit_value_t _value) {
    printf("Doorbell identify\n");
};
void lock_control_point(homekit_value_t value) {
    printf("Lock control point setter\n");
};
homekit_characteristic_t lock_current_state = HOMEKIT_CHARACTERISTIC_(
    LOCK_CURRENT_STATE,
    lock_state_unknown,
);
homekit_characteristic_t lock_target_state = HOMEKIT_CHARACTERISTIC_(
    LOCK_TARGET_STATE,
    lock_state_secured,
    .setter=lock_target_state_setter,
);
homekit_characteristic_t button_event = HOMEKIT_CHARACTERISTIC_(PROGRAMMABLE_SWITCH_EVENT, 0);

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_door_lock, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Lock"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "MNK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "ESP32ET01LK01"),
            HOMEKIT_CHARACTERISTIC(MODEL, "LockWithDoorbell"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, lock_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MECHANISM, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Lock"),
            &lock_current_state,
            &lock_target_state,
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MANAGEMENT, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(LOCK_CONTROL_POINT,
                .setter=lock_control_point
            ),
            HOMEKIT_CHARACTERISTIC(VERSION, "0.0.1"),
            NULL
        }),
        NULL
    }),
    HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_programmable_switch, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Door Bell"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "MNK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "ESP32ET01DB01"),
            HOMEKIT_CHARACTERISTIC(MODEL, "LockWithDoorbell"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, doorbell_identify),
            NULL
        }),
        HOMEKIT_SERVICE(DOORBELL, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Door Bell"),
            &button_event,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void on_wifi_ready() {
    printf("WiFi 初始化完成\n");
    homekit_server_init(&config);
};

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