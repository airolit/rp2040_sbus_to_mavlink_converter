#include <stdio.h>
#include "pico/stdlib.h"

#include "bsp/board.h"
#include "tusb.h"

#include "common/mavlink.h"
#include "common/common.h"

#define SBUS_MESSAGE_SIZE 25
#define NUM_SBUS_FIFOS 2

uint8_t sbus_data[SBUS_MESSAGE_SIZE];
uint32_t byte_index = 0;
uint16_t channels[16];

void parse_sbus_data()
{
    uint8_t *raw_channel_data = sbus_data + 1;

    // 16 channels of 11 bit data
    channels[0] = (uint16_t)((raw_channel_data[0] | raw_channel_data[1] << 8) & 0x07FF);
    channels[1] = (uint16_t)((raw_channel_data[1] >> 3 | raw_channel_data[2] << 5) & 0x07FF);
    channels[2] = (uint16_t)((raw_channel_data[2] >> 6 | raw_channel_data[3] << 2 | raw_channel_data[4] << 10) & 0x07FF);
    channels[3] = (uint16_t)((raw_channel_data[4] >> 1 | raw_channel_data[5] << 7) & 0x07FF);
    channels[4] = (uint16_t)((raw_channel_data[5] >> 4 | raw_channel_data[6] << 4) & 0x07FF);
    channels[5] = (uint16_t)((raw_channel_data[6] >> 7 | raw_channel_data[7] << 1 | raw_channel_data[8] << 9) & 0x07FF);
    channels[6] = (uint16_t)((raw_channel_data[8] >> 2 | raw_channel_data[9] << 6) & 0x07FF);
    channels[7] = (uint16_t)((raw_channel_data[9] >> 5 | raw_channel_data[10] << 3) & 0x07FF);
    channels[8] = (uint16_t)((raw_channel_data[11] | raw_channel_data[12] << 8) & 0x07FF);
    channels[9] = (uint16_t)((raw_channel_data[12] >> 3 | raw_channel_data[13] << 5) & 0x07FF);
    channels[10] = (uint16_t)((raw_channel_data[13] >> 6 | raw_channel_data[14] << 2 | raw_channel_data[15] << 10) & 0x07FF);
    channels[11] = (uint16_t)((raw_channel_data[15] >> 1 | raw_channel_data[16] << 7) & 0x07FF);
    channels[12] = (uint16_t)((raw_channel_data[16] >> 4 | raw_channel_data[17] << 4) & 0x07FF);
    channels[13] = (uint16_t)((raw_channel_data[17] >> 7 | raw_channel_data[18] << 1 | raw_channel_data[19] << 9) & 0x07FF);
    channels[14] = (uint16_t)((raw_channel_data[19] >> 2 | raw_channel_data[20] << 6) & 0x07FF);
    channels[15] = (uint16_t)((raw_channel_data[20] >> 5 | raw_channel_data[21] << 3) & 0x07FF);
}

void sbus_data_interrupt()
{
    while (uart_is_readable(uart1))
    {
        uint8_t ch = uart_getc(uart1);

        if (byte_index == 0 && ch != 0x0F)
        {
            return;
        }

        sbus_data[byte_index] = ch;
        byte_index++;

        if (byte_index == SBUS_MESSAGE_SIZE)
        {
            byte_index = 0;
            parse_sbus_data();
        }
    }
}

int main()
{
    stdio_init_all();

    uart_init(uart1, 115200);
    uart_set_baudrate(uart1, 100000);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 2, UART_PARITY_EVEN);
    uart_set_fifo_enabled(uart1, false);

    irq_set_exclusive_handler(UART1_IRQ, sbus_data_interrupt);
    irq_set_enabled(UART1_IRQ, true);

    uart_set_irq_enables(uart1, true, false);

    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);

    gpio_set_inover(5, GPIO_OVERRIDE_INVERT);

    const uint32_t num_sbus_bytes = 25;
    uint8_t buffer[num_sbus_bytes];

    uint32_t last_sbus_data_index = 0;

    board_init();
    tusb_init();

    // Get the current timestamp
    uint32_t last_timestamp = board_millis();

    while (true)
    {
        if (board_millis() - last_timestamp > 20)
        {

            int16_t throttle = channels[2];
            int16_t yaw = channels[3];
            int16_t pitch = channels[1];
            int16_t roll = channels[0];

            int16_t mid = 1024;
            int16_t max = 1684;
            int16_t min = 364;

            // Normalize throttle to a value between -1000 and 1000
            int16_t throttle_normalized = 500 - ((throttle - mid) * 1000 / (max - min));
            int16_t yaw_normalized = 2 * ((yaw - mid) * 1000 / (max - min));
            int16_t pitch_normalized = -2 * ((pitch - mid) * 1000 / (max - min));
            int16_t roll_normalized = 2 * ((roll - mid) * 1000 / (max - min));

            mavlink_message_t msg;
            mavlink_msg_manual_control_pack(1, 200, &msg, 0, pitch_normalized, roll_normalized, throttle_normalized, yaw_normalized, 0, 0, 0, 0, 0);

            static uint8_t mavlink_buffer[MAVLINK_MAX_PACKET_LEN] = {0};
            uint16_t len = mavlink_msg_to_send_buffer(mavlink_buffer, &msg);
            tud_cdc_write(mavlink_buffer, len);
            tud_cdc_write_flush();
            last_timestamp = board_millis();
        }
        tud_task();
    }
}