# T-Dongle S3

This repository provides example support for the [LilyGo T-Dongle
S3](https://github.com/Xinyuan-LilyGO/T-Dongle-S3) dev board which has:

* WiFi / BLE
* uSD card (hidden in the USB A connector!)
* RGB LED
* Color TFT LCD (ST7735, 80x160 0.96" IPS LCD)

## T-Dongle S3 Pin Configuration

| LED Pin | ESP32S3 IO Pin Number |
|---------|-----------------------|
| Data    | 40                    |
| Clock   | 39                    |

| LCD Pin   | ESP32S3 IO Pin Number        |
|-----------|------------------------------|
| CS        | 4                            |
| SDA       | 3                            |
| SCL       | 5                            |
| DC        | 2                            |
| Reset     | 1                            |
| Backlight | 38                           |

| TF / uSD Card Pin | ESP32S3 IO Pin Number |
|-------------------|-----------------------|
| D0                | 14                    |
| D1                | 17                    |
| D2                | 21                    |
| D3                | 18                    |
| CLK               | 12                    |
| CMD               | 16                    |

## Cloning

Since this repo contains a submodule, you need to make sure you clone it
recursively, e.g. with:

``` sh
git clone --recurse-submodules git@github.com:esp-cpp/t-dongle-s3
```

Alternatively, you can always ensure the submodules are up to date after cloning
(or if you forgot to clone recursively) by running:

``` sh
git submodule update --init --recursive
```

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output

