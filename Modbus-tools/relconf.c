/*****************************************************************
 * Relay configuration tool
 * 
 * This tool is used to set the device address of the BQTEK relay
 * cards via Modbus commands. The relay card has to be in "Settings
 * Mode" (power on with all DIP switches OFF). In this mode the 
 * device has the reserved slave address 255.
 * 
 * Author: Ondrej Wisniewski
 * 
 * Build with this command:
 * gcc relconf.c -o relconf -lmodbus
 * 
 * History:
 * 06/06/2018 - initial release
 * 
 * Copyright 2018
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Telegea.  If not, see http://www.gnu.org/licenses/.
 * 
 *****************************************************************/

// 
// Write Request:
// --------------
// FF      Slave addr 
// 06      Function Code (Write Single register)
// AA AA   Register Address  
// VV VV   Register Value
// 
// Response:
// ---------
// FF      Slave addr
// 06      Function Code (Write Single register)
// 00 XX   Register Address (2: slave addr, 3: baud rate)  
// VV VV   Register value
// 
//
// Read Request:
// -------------
// FF      Slave addr 
// 03      Function Code (Read holding registers)
// AA AA   Register Address  
// NN NN   Number of registers
// 
// Response:
// ---------
// FF      Slave addr
// 03      Function Code (Read holding registers)
// BB      Number of data bytes
// VV VV   Register value
//
//
// Config registers:
// -----------------
// 1       Device address (1 ... 254)
// 2       Baudrate (2400, 4800, ... 38400)
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <modbus/modbus.h>


#define VERSION       "0.1"

/* Debug mode */
#define DEBUG         0

#define SERIAL_PORT    "/dev/ttyAMA0"
#define BAUDRATE       9600
#define RTS_DELAY      10
#define REQ_FRAME_LEN  6
#define DATA_OFFSET_RD 3
#define DATA_OFFSET_WR 4


int main(int argc, char* argv[])
{
    modbus_t *mb;
    
    int fc;
    int reg_addr;
    int reg_val=1;
    int req_length;
    int rsp_length;
    uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];
    
    
    /**************************************************************
     * Parse input parameters
     **************************************************************/
    
    if (argc < 2)
    {
        printf("Relay sensor configuration tool, ver %s (using libmodbus %s)\n\n", VERSION, LIBMODBUS_VERSION_STRING);
        printf("usage: relconf <reg_addr> [<reg_val>]\n");
        return 0;
    }
    
    int i = 1;
    reg_addr = atoi(argv[i++]);
    if (argc > 2)
    {
        reg_val  = atoi(argv[i++]);
        fc = MODBUS_FC_WRITE_SINGLE_REGISTER;
    }
    else
    {
        fc = MODBUS_FC_READ_HOLDING_REGISTERS;
    }

    uint8_t raw_req[] = { 0xFF, fc, 0x00, reg_addr, reg_val>>8, reg_val };
    
    
    /**************************************************************
     * Initialize communication port
     **************************************************************/
    
    /* Create Modbus context */
    mb = modbus_new_rtu(SERIAL_PORT, BAUDRATE, 'N', 8, 1);
    if (mb == NULL) {
        printf("Unable to create the libmodbus context\n");
        return -1;
    }
    
    /* Set debug mode */
    if (DEBUG)
        modbus_set_debug(mb, TRUE);
    
    /* Set slave address */
    modbus_set_slave(mb, 0xFF);
    
    /* Connect to serial port */
    if (DEBUG)
        printf("Connecting to slave addr %d\n", 0xFF);
    if (modbus_connect(mb) == -1)
    {
        printf("Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(mb);
        return -1;
    }
    
    /* Set Modbus timeouts */
    modbus_set_response_timeout(mb, 2, 0); // 2s
    modbus_set_byte_timeout(mb, 0, 0);     // not used
    
    if (strstr(SERIAL_PORT, "USB") == NULL)
    {
        /* Enable RS485 direction control via RTS line */
        if (modbus_rtu_set_rts(mb, MODBUS_RTU_RTS_DOWN) == -1)
        {
            printf("Setting RTS mode failed: %s\n", modbus_strerror(errno));
            modbus_free(mb);
            return -1;
        }
        
        /* Set RTS control delay (before and after transmission) */
        if (RTS_DELAY>0)
        {
            if (modbus_rtu_set_rts_delay(mb, RTS_DELAY) == -1)
            {
                printf("Setting RTS delay failed: %s\n", modbus_strerror(errno));
                modbus_free(mb);
                return -1;
            }
        }
        if (DEBUG)
            printf("RTS delay is %dus\n", modbus_rtu_get_rts_delay(mb));
    }
    
    
    /**************************************************************
     * Perform request
     **************************************************************/
    
    req_length = modbus_send_raw_request(mb, raw_req, REQ_FRAME_LEN * sizeof(uint8_t));
    rsp_length = modbus_receive_confirmation(mb, rsp);

    if (fc == MODBUS_FC_READ_HOLDING_REGISTERS)
    {
        reg_val = rsp[DATA_OFFSET_RD]<<8;
        reg_val |= rsp[DATA_OFFSET_RD+1];
    }
    else
    {
        reg_val = rsp[DATA_OFFSET_WR]<<8;
        reg_val |= rsp[DATA_OFFSET_WR+1];
    }
    
    //printf("req_length:%d, rsp_length:%d\n", req_length, rsp_length);
    if (rsp_length != -1)
    {
        printf("reg %d: 0x%04X (%d)\n", reg_addr, reg_val, reg_val);
    }
    else
    {
        printf("ERROR performing Modbus request\n");
    }
    
    
    /**************************************************************
     * Cleanup and exit
     **************************************************************/
    modbus_close(mb);
    modbus_free(mb);
    
    return 0;    
}
