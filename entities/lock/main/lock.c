#include <stdio.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include "esp_http_client.h"
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
homekit_characteristic_t lock_current_state;
homekit_characteristic_t lock_target_state;
TaskHandle_t updateStateTask;
const int doorbell_gpio = 33;
const int lock_gpio = 32;
const char homebridge_camera_doorbell_trigger_url[] = "http://192.168.50.43:12580/doorbell?acs";
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

esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            printf("HTTP_EVENT_ERROR \n");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            printf("HTTP_EVENT_ON_CONNECTED \n");
            break;
        case HTTP_EVENT_HEADER_SENT:
            printf("HTTP_EVENT_HEADER_SENT \n");
            break;
        case HTTP_EVENT_ON_HEADER:
            printf("HTTP_EVENT_ON_HEADER \n");
            printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            printf("HTTP_EVENT_ON_DATA, len=%d\n", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            printf("HTTP_EVENT_ON_FINISH\n");
            break;
        case HTTP_EVENT_DISCONNECTED:
            printf("HTTP_EVENT_DISCONNECTED\n");
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
};


void update_doorbell_state() {
    while (true){
        int gpio_level_doorbell = gpio_get_level(doorbell_gpio);
        if (gpio_level_doorbell == 1) {
            esp_http_client_config_t config = {
                .url = homebridge_camera_doorbell_trigger_url,
                .event_handler = _http_event_handle,
            };
            esp_http_client_handle_t client = esp_http_client_init(&config);
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
                printf("Status = %d, content_length = %d\n",
                    esp_http_client_get_status_code(client),
                    esp_http_client_get_content_length(client));
            }
            esp_http_client_cleanup(client);
            vTaskDelay(100);
        }
    }
}

void lock_lock() {
    lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
    vTaskDelete(NULL);
}

void lock_unlock() {
    printf("接收到开锁命令\n");
    printf("处理 GPIO 引脚事件\n");
    gpio_set_level(lock_gpio, 1);
    lock_current_state.value = HOMEKIT_UINT8(lock_state_unsecured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    lock_target_state.value = HOMEKIT_UINT8(lock_state_secured);
    lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
    homekit_characteristic_notify(&lock_target_state, lock_target_state.value);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
    vTaskDelete(NULL);
};

void lock_target_state_setter(homekit_value_t value) {
    lock_target_state.value = value;
    if (value.int_value == 0) {
        xTaskCreate(lock_unlock, 'LockUnlock', 4096, NULL, 5, NULL);
    } else {
        xTaskCreate(lock_lock, 'LockLock', 4096, NULL, 5, NULL);
    }
};
void lock_identify(homekit_value_t _value) {
    printf("Lock identify\n");
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
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void control_init() {
    printf("初始化门铃检测引脚\n");
    gpio_reset_pin(doorbell_gpio);
    gpio_pad_select_gpio(doorbell_gpio);
    gpio_set_direction(doorbell_gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(doorbell_gpio, GPIO_PULLDOWN_ONLY);

    printf("初始化门锁控制引脚\n");
    gpio_reset_pin(lock_gpio);
    gpio_set_direction(lock_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(lock_gpio, 0);

    printf("控制系统和传感器初始化完成\n");
}

void update_state_init() {
    xTaskCreate(update_doorbell_state, 'UpdateDoorbellState', 4096, NULL, 5, NULL);
}

void on_wifi_ready() {
    printf("WiFi 初始化完成\n");
    homekit_server_init(&config);
    control_init();
    update_state_init();
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