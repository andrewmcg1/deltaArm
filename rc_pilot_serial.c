#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <rc/uart.h>

#include "rc_pilot_serial.h"

#define DATA_START_BYTE_1 0x81
#define DATA_START_BYTE_2 0xA1

enum _ParseState
{
    START_BYTE_1,
    START_BYTE_2,
    DATA,
    CHECKSUM,
    END
};
typedef enum _ParseState ParseState;


int serial_init(int bus, int baudrate, float timeout)
{
    if(rc_uart_init(bus, baudrate, timeout, 0, 1, 0))// || rc_uart_flush(bus))
        return -1;
    else
        return 0;

}

void __read_byte(int bus)
{
    static __uint8_t buf[256];
    static __uint8_t current_byte;
    static ParseState parseState = START_BYTE_1;
    static unsigned int index = 0;

    rc_uart_read_bytes(bus, &current_byte, 1);

    switch (parseState)
    {
        case START_BYTE_1:
            if (current_byte == DATA_START_BYTE_1)
                parseState = START_BYTE_2;
            break;

        case START_BYTE_2:
            if (current_byte == DATA_START_BYTE_2)
            {
                parseState = DATA;
            }
            else
                parseState = START_BYTE_1;  // resets state machine
            break;

        case DATA:
            buf[index] = current_byte;
            index++;
            if (index == sizeof(data_packet_t) - 1)
            {
                parseState = START_BYTE_1;
                index = 0;

                data_packet = *(data_packet_t*)buf;
                if(data_packet.id == DRONE_ID)
                    drone_data_packet = data_packet;
                else if(data_packet.id == DELTA_ID)
                    delta_data_packet = data_packet;
            }

            break;
    }
}

void __read_all(int bus)
{
    __uint8_t buf[64];
    __uint8_t current_byte;
    __uint8_t checksum1 = 0;
    __uint8_t checksum2 = 0;
    __uint16_t checksum_calculated = 0;
    __uint16_t checksum_received = 0;

    int parseState = START_BYTE_1;

    while(parseState != END && rc_uart_bytes_available(bus) > 0)
    {
        switch (parseState)
        {
            case START_BYTE_1:
                rc_uart_read_bytes(bus, &current_byte, 1);
                if (current_byte == DATA_START_BYTE_1)
                {
                    parseState = START_BYTE_2;
                }
                break;
            case START_BYTE_2:
                rc_uart_read_bytes(bus, &current_byte, 1);
                if (current_byte == DATA_START_BYTE_2)
                {
                    parseState = DATA;
                }
                else
                    parseState = START_BYTE_1;  // resets state machine
                break;
            case DATA:
                rc_uart_read_bytes(bus, buf, sizeof(data_packet_t) + 2);
                parseState = CHECKSUM;
                break;
            case CHECKSUM:
                for(int i = 0; i < sizeof(data_packet_t); i++)
                {
                    checksum1 += buf[i];
                    checksum2 += checksum1;
                }

                checksum_calculated = (uint16_t)((checksum2 << 8) | checksum1);
                checksum_received = (uint16_t)((buf[sizeof(data_packet_t) + 1] << 8) | buf[sizeof(data_packet_t)]);

                if(checksum_calculated == checksum_received)
                {
                    data_packet = *(data_packet_t*)buf;
                    if(data_packet.id == DRONE_ID)
                        drone_data_packet = data_packet;
                    if(data_packet.id == DELTA_ID)
                        delta_data_packet = data_packet;

                    parseState = END;
                }
                else
                {
                    printf("Invalid Checksum: %x != %x\n", checksum_calculated, checksum_received);
                    parseState = START_BYTE_1;
                }

                //printf("ID: %1d | State: %d | X: %.4f | Y: %.4f | Z: %.4f\n", data_packet.id, data_packet.state, data_packet.x, data_packet.y, data_packet.z);

                break;
            
        }
    }
}

int serial_receive(int bus)
{
    int temp = rc_uart_bytes_available(bus);
    //printf("\rBytes: %4d", temp);
    if(temp > 0)
    {
        __read_all(bus);
        return 0;
    }
    else
        return -1;
}