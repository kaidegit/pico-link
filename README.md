# 使用树莓派Pico制作的DAPLink调试器

使用树莓派Pico（RP2040）实现的DAPLink

## IO配置

具体可查看Core/inc/io_config.h

SWDIO_IN通过100R与SWDIO_OUT相连

| IO标识 | IO编号 |
| ------ | ------ |
| SWCLK  | GPIO19 |
| SWDIO  | GPIO18 |
| RESET  | GPIO17 |
| TX     | 20     |
| RX     | 21     |





