//
// Created by yekai on 2021/12/29.
//

#include <hardware/gpio.h>
#include <hardware/irq.h>
#include "virtual_UART.h"
#include "hardware/uart.h"

// USB->LINK回调
void tud_cdc_rx_cb(uint8_t itf) {
    char buf[CFG_TUD_CDC_RX_BUFSIZE] = {0};
    tud_cdc_read(buf, CFG_TUD_CDC_RX_BUFSIZE);
    uart_puts(PICO_LINK_UART_ID, buf);
}

// MCU->LINK回调
void on_uart_rx() {
    while (uart_is_readable(PICO_LINK_UART_ID)) {
        uint8_t ch = uart_getc(PICO_LINK_UART_ID);
        tud_cdc_write_char(ch);
    }
    tud_cdc_write_flush();
}

void VCOM_Init() {
    cdc_line_coding_t uart_config;
    tud_cdc_get_line_coding(&uart_config);
    uart_init(PICO_LINK_UART_ID, uart_config.bit_rate);
    gpio_set_function(PICO_LINK_UART_RX, GPIO_FUNC_UART);
    gpio_set_function(PICO_LINK_UART_TX, GPIO_FUNC_UART);
    irq_set_exclusive_handler(PICO_LINK_UART_IRQ, on_uart_rx);
    irq_set_enabled(PICO_LINK_UART_IRQ, true);
    uart_set_irq_enables(PICO_LINK_UART_ID, true, false);
}

void VCOM_SendString(char *str) {
    while (*str) {
        tud_cdc_write_char(*str++);
    }
    tud_cdc_write_char('\r');
    tud_cdc_write_char('\n');
    tud_cdc_write_flush();
}

// 检测到上位机串口助手开启串口回调
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    VCOM_Init();
}
