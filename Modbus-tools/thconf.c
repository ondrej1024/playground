/*****************************************************************
 * T/H sensor configuration tool
 * 
 * This tool is used to change the baudrate and slave address 
 * settings of the Modbus T/H sensor PKTH100B. 
 * 
 * This sensor is available from chinese online stores under 
 * various names and from different manufacturers. This tool 
 * might work also for other similar sensors or it might not.
 * 
 * In order to change the communication settings of the sensor, a
 * NON STANDARD extension of the Modbus "Write Single register"
 * message is used.
 * 
 * Author: Ondrej Wisniewski
 * 
 * Build with this command:
 * gcc thconf.c -o thconf -lmodbus
 * 
 * History:
 * 27/06/2017 - initial release
 * 
 * Copyright 2017
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

// Set Slave addr and baud rate (message dedscription)
// ===================================================
// 
// Request:
// --------
// 01      Slave addr
// 06      Function Code (Write Single register)
// 00 00   Start Address (0)  
// 00 01   Register value (1)
// 02      Number of additional data bytes (Non standard extension)
// AA      New slave addr (Non standard extension)
// BB      New baud rate (Non standard extension)
// 
// Response:
// --------
// 01      Slave addr
// 06      Function Code (Write Single register)
// 00 00   Start Address (0)  
// 00 01   Register value
// 
// 
// Slave addr AA:
// 01-F7
// 
// Baud rate BB:
// 03: 1200
// 04: 2400
// 05: 4800
// 06: 9600
// 07: 19200


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <modbus/modbus.h>


#define VERSION       "0.1"

/* Debug mode */
#define DEBUG         0

#define SERIAL_PORT   "/dev/ttyAMA0"
#define BAUDRATE      9600
#define RTS_DELAY     10
#define REQ_FRAME_LEN 9
#define RSP_FRAME_LEN 6

uint8_t check_baudrate(int baudrate)
{
    uint8_t br;
    
    switch (baudrate)
    {
        case 1200:
            br=3;
            break;
        case 2400:
            br=4;
            break;
        case 4800:
            br=5;
            break;
        case 9600:
            br=6;
            break;
        case 19200:
            br=7;
            break;
        default:
            br=0;
    }
    return br;
}

int main(int argc, char* argv[])
{
    modbus_t *mb;
    
    int baudrate;
    int new_baudrate;
    int slave_addr;   
    int new_slave_addr;   
    int req_length;
    uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];
    uint8_t br_code;
    
    
    /**************************************************************
     * Parse input parameters
     **************************************************************/
    
    if (argc < 5)
    {
        printf("TH sensor configuration tool, ver %s (using libmodbus %s)\n\n", VERSION, LIBMODBUS_VERSION_STRING);
        printf("usage: thconf <baudrate> <slave_addr> <new_baudrate> <new_slave_addr>\n");
        printf("   baudrate, new_baudrate:     1200,2400,4800,9600,19200\n");
        printf("   slave_addr, new_slave_addr: 1..247\n");
        return 0;
    }
    
    int i = 1;
    baudrate       = atoi(argv[i++]);
    slave_addr     = atoi(argv[i++]);
    new_baudrate   = atoi(argv[i++]);
    new_slave_addr = atoi(argv[i++]);
    
    if (check_baudrate(baudrate) == 0)
    {
        printf("Invalid baudrate %d\n", baudrate);
        return -1;
    }
    
    if ((br_code=check_baudrate(new_baudrate)) == 0)
    {
        printf("Invalid new baudrate %d\n", new_baudrate);
        return -1;
    }
    
    if ((slave_addr<1) || (slave_addr>247))
    {
        printf("Invalid slave address %d\n", slave_addr);
        return -1;
    }
    
    if ((new_slave_addr<1) || (new_slave_addr>247))
    {
        printf("Invalid new slave address %d\n", new_slave_addr);
        return -1;
    }
    
    uint8_t raw_req[] = { slave_addr, MODBUS_FC_WRITE_SINGLE_REGISTER, 0x00, 0x00, 0x00, 0x01, 0x02, new_slave_addr, br_code };
    
    
    /**************************************************************
     * Initialize communication port
     **************************************************************/
    
    /* Create Modbus context*/
    mb = modbus_new_rtu(SERIAL_PORT, baudrate, 'N', 8, 1);
    if (mb == NULL) {
        printf("Unable to create the libmodbus context\n");
        return -1;
    }
    
    /* Set debug mode */
    if (DEBUG)
        modbus_set_debug(mb, TRUE);
    
    /* Set slave address */
    modbus_set_slave(mb, slave_addr);
    
    /* Connect to serial port */
    if (DEBUG)
        printf("Connecting to slave addr %d\n", slave_addr);
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
    modbus_receive_confirmation(mb, rsp);
    
    if (memcmp(raw_req, rsp, RSP_FRAME_LEN) == 0)
    {
        printf("Successfully changed sensor configuration\n");
        if (baudrate != new_baudrate)
           printf("New baudrate: %d\n", new_baudrate);
        if (slave_addr != new_slave_addr)
           printf("New slave address: %d\n", new_slave_addr);
    }
    else
    {
        printf("ERROR changing sensor configuration, check parameters\n");
    }
    
    
    /**************************************************************
     * Cleanup and exit
     **************************************************************/
    modbus_close(mb);
    modbus_free(mb);
    
    return 0;    
}
