# HomeKit entities for ESP32

> Based on [maximkulkin/esp-homekit-demo](https://github.com/maximkulkin/esp-homekit-demo).
> It's a fully customized project, works with specified devices.

## Current entities

- [x] Door with an angle sensor
- [ ] Window with an angle sensor and a rain sensor
- [ ] Lock with an electronic doorbell

## Build instructions

1. Initialize and sync all submodules recursively:

    ```shell
    git submodule update --init --recursive
    ```

2. Copy `wifi.h.sample` → `wifi.h` and edit it with correct WiFi SSID and password.

3. Install [esp-idf](https://github.com/espressif/esp-idf).

4. Configure project:
    
    > Check "Serial flasher config → Default serial port"

    ```shell
    make menuconfig
    ```

5. Build, flash and monitor:

    ```shell
    make erase_flash && make all && make flash
    ```
