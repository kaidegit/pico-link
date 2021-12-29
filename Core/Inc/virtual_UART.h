//
// Created by yekai on 2021/12/29.
//

#ifndef PICO_LINK_VIRTUAL_UART_H
#define PICO_LINK_VIRTUAL_UART_H

#include "main.h"
#include "tusb.h"

void VCOM_SendString(char *str);

void VCOM_Init();

#endif //PICO_LINK_VIRTUAL_UART_H
